clear all;
close all;
clc;

% Parameters
g = 9.81;              % Gravity (m/s^2)
m = 1.0;               % Mass of quadcopter (kg)
Ts = 0.1;              % Sample time (s)
nx = 6;                % Number of states: [px, vx, py, vy, pz, vz]
ny = 3;                % Number of outputs: [px, py, pz]
nu = 3;                % Number of inputs: [phi (roll), theta (pitch), thrust_delta/m]
P = 10;                % Prediction horizon
M = 10;                % Control horizon (set equal to P for full optimization)
sim_steps = 200;       % Number of simulation steps

% Adjusted Weights
Q = diag([1, 1, 1]);   % Reduced output weights for less aggressive tracking
R = diag([0.5, 0.5, 0.5]); % Increased input weights for smoother control
S = diag([0.5, 0.5, 0.5]); % Reduced rate weights to allow more flexibility

% Constraints (relaxed)
max_tilt = pi/4;       % Increased max tilt to 45 deg
min_thrust = 0;        % Min thrust (N)
max_thrust = 3 * m * g; % Increased max thrust
thrust_eq = m * g;     % Equilibrium thrust
max_alt = 15;          % Increased max altitude
min_alt = 0;           % Min altitude
du_max = [0.2; 0.2; g*0.2]; % Doubled rate of change limits
du_min = -du_max;      % Min rate of change

% Input bounds
u_max = [max_tilt; max_tilt; (max_thrust - thrust_eq)/m];
u_min = [-max_tilt; -max_tilt; (min_thrust - thrust_eq)/m];

% Output bounds
y_max = [Inf; Inf; max_alt];
y_min = [-Inf; -Inf; min_alt];

% Initial state
x0 = zeros(nx, 1);

% Target reference
ref = [10; 10; 5] * ones(1, P);

% Previous input
u_prev = zeros(nu, 1);

%% Quadcopter Linear Model
A_cont = [0 1 0 0 0 0;
          0 0 0 0 0 0;
          0 0 0 1 0 0;
          0 0 0 0 0 0;
          0 0 0 0 0 1;
          0 0 0 0 0 0];
B_cont = [0   0    0;
          0   g    0;
          0   0    0;
          -g  0    0;
          0   0    0;
          0   0    1];
C = [1 0 0 0 0 0;
     0 0 1 0 0 0;
     0 0 0 0 1 0];
D = zeros(ny, nu);

sys_cont = ss(A_cont, B_cont, C, D);
sys_disc = c2d(sys_cont, Ts);
Ad = sys_disc.A;
Bd = sys_disc.B;
Cd = sys_disc.C;

% Precompute Prediction Matrices
Q_bar = kron(eye(P), Q);
R_bar = kron(eye(M), R);
S_bar = kron(eye(M), S);

F = zeros(P*ny, nx);
Phi = zeros(P*ny, M*nu);
A_pow = eye(nx);
for i = 1:P
    A_pow = Ad * A_pow;
    F((i-1)*ny+1:i*ny, :) = Cd * A_pow;
    for j = 1:min(i, M)
        pow = Ad ^ (i - j);
        Phi((i-1)*ny+1:i*ny, (j-1)*nu+1:j*nu) = Cd * pow * Bd;
    end
end

Du = zeros(M*nu, M*nu);
for i = 1:M
    Du((i-1)*nu+1:i*nu, (i-1)*nu+1:i*nu) = eye(nu);
    if i > 1
        Du((i-1)*nu+1:i*nu, (i-2)*nu+1:(i-1)*nu) = -eye(nu);
    end
end
d0 = repmat(-u_prev, M, 1);

% QP Setup
H = Phi' * Q_bar * Phi + R_bar + Du' * S_bar * Du;
H = (H + H') / 2 + 1e-6 * eye(size(H)); % Regularization for stability
% f will be computed in loop

% Constraints Setup
u_min_rep = kron(ones(M,1), u_min);
u_max_rep = kron(ones(M,1), u_max);
du_min_rep = kron(ones(M,1), du_min) - d0;
du_max_rep = kron(ones(M,1), du_max) - d0;
A_du = [Du; -Du];
b_du = [du_max_rep; -du_min_rep];
y_min_rep = kron(ones(P,1), y_min) - F * x0;
y_max_rep = kron(ones(P,1), y_max) - F * x0;
A_y = [Phi; -Phi];
b_y = [y_max_rep; -y_min_rep];
A_ineq = [eye(M*nu); -eye(M*nu); A_du; A_y];
b_ineq = [u_max_rep; -u_min_rep; b_du; b_y];

% Simulation Loop
x = x0;
x_history = zeros(nx, sim_steps+1);
u_history = zeros(nu, sim_steps);
y_history = zeros(ny, sim_steps);
x_history(:,1) = x;

for k = 1:sim_steps
    ref_vec = ref(:);
    free_resp = F * x;
    err = free_resp - ref_vec;
    f = 2 * (Phi' * Q_bar * err + Du' * S_bar * d0)';
    d0 = zeros(M*nu, 1);
    d0(1:nu) = -u_prev;
    y_min_rep = kron(ones(P,1), y_min) - free_resp;
    y_max_rep = kron(ones(P,1), y_max) - free_resp;
    b_y = [y_max_rep; -y_min_rep];
    du_min_rep = kron(ones(M,1), du_min) - d0;
    du_max_rep = kron(ones(M,1), du_max) - d0;
    b_du = [du_max_rep; -du_min_rep];
    b_ineq = [u_max_rep; -u_min_rep; b_du; b_y];
    
    options = optimoptions('quadprog', 'Display', 'off', 'Algorithm', 'interior-point-convex', 'TolFun', 1e-6, 'TolCon', 1e-6');
    [u_seq, ~, exitflag] = quadprog(H, f, A_ineq, b_ineq, [], [], [], [], [], options);
    
    if exitflag <= 0
        warning('QP not solved optimally at step %d', k);
        u = zeros(nu, 1);
    else
        u = u_seq(1:nu);
    end
    
    x = Ad * x + Bd * u;
    x_history(:, k+1) = x;
    u_history(:, k) = u;
    y_history(:, k) = Cd * x;
    u_prev = u;
end

% Plots
t = (0:sim_steps) * Ts;
figure(1);
subplot(3,1,1); plot(t(1:end-1), y_history(1,:)); ylabel('px (m)'); title('Positions');
subplot(3,1,2); plot(t(1:end-1), y_history(2,:)); ylabel('py (m)');
subplot(3,1,3); plot(t(1:end-1), y_history(3,:)); ylabel('pz (m)'); xlabel('Time (s)');

figure(2);
subplot(3,1,1); plot(t(1:end-1), u_history(1,:)); ylabel('phi (rad)'); title('Inputs');
subplot(3,1,2); plot(t(1:end-1), u_history(2,:)); ylabel('theta (rad)');
subplot(3,1,3); plot(t(1:end-1), m * (u_history(3,:) + g)); ylabel('thrust (N)'); xlabel('Time (s)');

disp('Simulation complet.');

