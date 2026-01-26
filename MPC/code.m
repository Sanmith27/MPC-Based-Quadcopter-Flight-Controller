%% Quadcopter state-space model
clc;
clear all;
close all;
%% constants
m=0.70; % mass kg
g=9.81; 

Ix = 9.753E-3;
Iy = 9.753E-3;
Iz = 15.078E-3; 

dx=0; dy=0; dz=0; % disturbances
kx=0.04; ky=0.04; kz=1.05; 

% States Considered
% x=[x y z vx vy vz roll pitch yaw wr wp wy];

% Control Inputs
% u=[T tr tp ty];

%% State space model simplified
% sin(x) -> x; cos(x) -> 1; tan(x) -> x; considering no disturbance
syms x1 x2 x3 x4 x5 x6 x7 x8 x9 x10 x11 x12
syms u1 u2 u3 u4
xdot1=x4;
xdot2=x5;
xdot3=x6;
xdot4=(u1/m)*(x8+x9*x7)-kx*x4/m;
xdot5=(u1/m)*(x9*x8-x7)-ky*x5/m;
xdot6=(u1/m)-g-kz*x6/m;
xdot7=x10+x11*x7*x8+x12*x8;
xdot8=x11-x12*x7;
xdot9=x7*x11+x12;
xdot10=(u2/Ix)-((Iy-Iz)/Ix)*x11*x12;
xdot11=(u3/Iy)-((Iz-Ix)/Iy)*x10*x12;
xdot12=(u4/Iz)-((Ix-Iy)/Iz)*x10*x11;
xdot=[xdot1 xdot2 xdot3 xdot4 xdot5 xdot6 xdot7 xdot8 xdot9 xdot10 xdot11 xdot12].';
x=[x1 x2 x3 x4 x5 x6 x7 x8 x9 x10 x11 x12].';
u=[u1 u2 u3 u4].';
y=[x1 x2 x3 x9].';

[x_size,aux]=size(x);
[y_size,aux]=size(y);
[u_size,aux]=size(u);


%% Equilibrium points (linearization about hover)
% ue = [m*g; 0; 0; 0]  (thrust balances weight), other inputs zero
% xe = [x1; x2; x3; 0;0;0; 0;0;0; 0;0;0] (positions arbitrary, velocities/angles zero)

% --- substitution  ---

sym_list = [x1 x2 x3 x4 x5 x6 x7 x8 x9 x10 x11 x12 u1 u2 u3 u4];

replacement_list = [x1, x2, x3, 0, 0, 0, 0, 0, 0, 0, 0, 0, m*g, 0, 0, 0];

% Jacobians
JA = jacobian(xdot, x.');    % 12x12
JB = jacobian(xdot, u.');    % 12x4
JC = jacobian(y, x.');       % ny x 12

% substitute equilibrium values
A_sym = subs(JA, sym_list, replacement_list);
B_sym = subs(JB, sym_list, replacement_list);
C_sym = subs(JC, sym_list, replacement_list);

% convert to numeric matrices
A = double( subs(A_sym, [x1 x2 x3], [0 0 0]) );
B = double( subs(B_sym, [x1 x2 x3], [0 0 0]) );
C = double( subs(C_sym, [x1 x2 x3], [0 0 0]) );

%% Discrete state-space model
fs = 50;
Ts = 1/fs;
sysc = ss(A,B,C,zeros(size(C,1),size(B,2))); % D matrix sized properly
sysd = c2d(sysc,Ts);

Am = sysd.A;
Bm = sysd.B;
Cm = sysd.C;

% Dm: disturbance matrix - set to zeros with matching output rows
Dm = zeros(size(Cm,1),1);

% repeat
fs=50; % 100 Hz
Ts=1/fs; 
sysc=ss(A,B,C,0);
sysd=c2d(sysc,Ts);

Am=sysd.A;
Bm=sysd.B;
Cm=sysd.C;
Dm=zeros(x_size,1);

%% MPC
%% Initialization
N_sim=10*fs; %% samples = seconds * frequency

% tuning parameters
Nc=10;  % control horizon
Np=50; % prediction horizon
R = 0.001;   % control weighting

% Init control, reference and output signal
u=zeros(u_size,1);  
y=zeros(y_size,1); 

rx=0*ones(1,N_sim);
ry=0*ones(1,N_sim);
rz=0*ones(1,N_sim);
ryaw=0*ones(1,N_sim);

xm_vector=[];
u_vector=[];
deltau_vector=[];
y_vector=[];

% Initialize system states
xm=zeros(x_size,1); % states vector
Xf=zeros(x_size+y_size,1); % augmented incremental state [deltax y]'

% Get the augmented incremental model and parameters for incremental trajectory control
[GG, GF, GR, A, B, C , GD, F, G]=mimo_gain(Am,Bm,Cm,Nc,Np,Dm); 

% Constant part of incremental control law
f1=GG+R*eye(Nc*u_size,Nc*u_size); % E for cost function J=xEx'+x'F

% Constraint matrix M
ymax=[0.8 1 2];  % output max limits
ymin=[-0.3 -1 -1.5];  % output min limits

% must test
deltaumax=500;
deltaumin=-500;

% for negative u, motor direction needs inversion
% u1 must be limited (divide by 2) because it controls 4 motors instead of 2
umax=500;
umin=-500;

M_output=[];
[gm,gn]=size(G);
aux=eye(gn);

M_deltau=[aux(1,:);-aux(1,:);
    aux(2,:);-aux(2,:);
    aux(3,:);-aux(3,:);
    aux(4,:);-aux(4,:)];

M_u=[aux(1,:);-aux(1,:);
    aux(2,:);-aux(2,:);
    aux(3,:);-aux(3,:);
    aux(4,:);-aux(4,:)];

M=[M_output; M_deltau; M_u];

%% LOOP
for k=1:N_sim;  
    %% Generate the Reference trajectory
    rx(k)=sin(0.05*k);    
    ry(k)=cos(0.05*k);       
    rz(k)=0.01*k;    
    ryaw(k)=45*pi/180;
    
    r=[rx; ry; rz; ryaw];
    
    %% Calculate DeltaU
    f2=GR*r(:,k)-GF*Xf; % Second part of control law
    DeltaU=inv(f1)*f2;  % Without constraints
    
    %% Calculate DeltaU with constraints
    gamma_output=[];
    gamma_deltau=[deltaumax;-deltaumin;
        deltaumax;-deltaumin;
        deltaumax;-deltaumin;
        deltaumax;-deltaumin];
    gamma_u=[umax-u(1);0+u(1);
        umax-u(2);-umin+u(2);
        umax-u(3);-umin+u(3);
        umax-u(4);-umin+u(4)];
    
    gamma=[gamma_output; gamma_deltau; gamma_u];
    
    %DeltaU=optim(f1,-f2,M,gamma,DeltaU); % Hildreth's algorithm for constraints
    
    deltau=DeltaU(1:4);  % receding horizon → take first control input
    u=u+deltau;          % u(k)=u(k-1)+deltau(k)
    u=u-[m*g;0;0;0];     % equilibrium compensation
    
    %% update State    
    xm_old=xm; 
    xm=Am*xm+Bm*u; 
    y=Cm*xm;    
    %y=y+rand()*[0.1;0.2;0.2;0.1]; % disturbance noise 
    Xf=[xm-xm_old;y]; 
    
    xm_vector=[xm_vector xm];
    y_vector=[y_vector y];
    u_vector=[u_vector u];
    deltau_vector=[deltau_vector deltau];
  
end

%% ---------------------- Plots ----------------------------

p = 0:N_sim-1;

%% -------- Output (y) vs Reference (r) -----------
figure;
for i = 1:4
    subplot(4,1,i)
    plot(p, y_vector(i,:), 'LineWidth', 2);
    hold on; grid on;
    plot(p, r(i,:), '--', 'LineWidth', 1.5);

    switch i
        case 1, title('X Position Tracking'); ylabel('x (m)');
        case 2, title('Y Position Tracking'); ylabel('y (m)');
        case 3, title('Z Position Tracking'); ylabel('z (m)');
        case 4, title('Yaw Tracking'); ylabel('yaw (rad)');
    end
    legend('Actual', 'Reference');
end
xlabel('Time step (k)');

%% -------- Control Input --------------------
figure;
for i = 1:4
    subplot(4,1,i)
    plot(p, u_vector(i,:), 'LineWidth', 2);
    grid on;

    switch i
        case 1, title('Control Input u1 (Total Thrust)'); 
        case 2, title('Control Input u2 (Roll Torque)'); 
        case 3, title('Control Input u3 (Pitch Torque)'); 
        case 4, title('Control Input u4 (Yaw Torque)');
    end

    ylabel(['u' num2str(i)]);
end
xlabel('Time step (k)');

%% -------- Control Increment) -----------
figure;
for i = 1:4
    subplot(4,1,i)
    plot(p, deltau_vector(i,:), 'LineWidth', 2);
    grid on;
    ylabel(['\Delta u' num2str(i)]);

    switch i
        case 1, title('Change in Total Thrust');
        case 2, title('Change in Roll Torque');
        case 3, title('Change in Pitch Torque');
        case 4, title('Change in Yaw Torque');
    end
end
xlabel('Time step (k)');

%% -------- 12 states --------------------
figure;
for i = 1:12
    subplot(4,3,i)
    plot(p, xm_vector(i,:), 'LineWidth', 1.5);
    grid on;
    ylabel(['x' num2str(i)]);

    if i <= 3
        title(['Position State x' num2str(i)]);
    elseif i <= 6
        title(['Velocity State x' num2str(i)]);
    elseif i <= 9
        title(['Angle State x' num2str(i)]);
    else
        title(['Angular Rate State x' num2str(i)]);
    end
end
xlabel('Time step (k)');

%% -------- 3D Trajectory Plot -------------------------
figure;
plot3(y_vector(1,:), y_vector(2,:), y_vector(3,:), 'LineWidth', 2);
hold on; grid on;
plot3(r(1,:), r(2,:), r(3,:), '--', 'LineWidth', 2);
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('3D Flight Trajectory');

legend('Actual Path', 'Reference Path');
