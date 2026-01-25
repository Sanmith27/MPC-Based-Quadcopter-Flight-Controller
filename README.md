# MPC-Based Quadcopter Flight Controller

## Overview
This project presents the design and simulation of a quadcopter flight control system based on Model Predictive Control (MPC).  
The objective is to achieve stable, smooth, and constraint-aware trajectory tracking for a nonlinear, underactuated quadcopter model.  
The controller is evaluated through MATLAB/Simulink simulations using multiple reference trajectories and disturbance scenarios.

---

## System Architecture
The quadcopter is modeled using a full nonlinear Newton–Euler formulation with 12 state variables representing position, velocity, Euler angles, and angular rates.  
The control inputs consist of total thrust and three body-axis torques.  
The nonlinear model is linearized around the hover equilibrium to obtain a discrete-time state-space representation suitable for MPC.

---

## Mathematical Modeling
- Developed a complete nonlinear quadcopter dynamic model using Newton–Euler equations  
- Defined translational and rotational dynamics in inertial and body frames  
- Incorporated thrust and torque generation from individual rotor speeds  
- Linearized the nonlinear model about hover using small-angle approximations  
- Formulated a 12-state representation for controller design  

The model captures strong coupling between translational and rotational motion and explicitly represents actuator limitations.

---

## MPC Design and Control Strategy
- Implemented a constrained Model Predictive Controller for position and attitude regulation  
- Selected prediction and control horizons to balance tracking accuracy and computational feasibility  
- Designed a quadratic cost function penalizing tracking error and control effort  
- Enforced physical constraints on:
  - Motor thrust
  - Body torques
  - Attitude angles
  - Angular rates  

At each control step, the MPC solves a constrained optimization problem and applies the first control action using a receding-horizon strategy.

---

## Simulation Setup
- Implemented the quadcopter model and MPC controller in MATLAB/Simulink  
- Tested with multiple reference trajectories:
  - Circular motion in the X–Y plane
  - Linear altitude changes
  - Constant yaw tracking  
- Introduced initial offsets and disturbances to evaluate robustness  

---

## Results and Observations
- Stable flight maintained throughout all simulations  
- Accurate tracking of 3D reference trajectories with minimal steady-state error  
- Smooth control inputs after initial transient response  
- Motor speeds converge to realistic hover values (~900–1200 rad/s)  
- Attitude angles and angular rates exhibit well-damped behavior  

The results demonstrate that MPC provides smoother trajectory tracking, improved constraint handling, and better stability compared to conventional controllers.

---

## Key Takeaways
- MPC effectively handles multivariable coupling and actuator constraints  
- Constraint-aware optimization prevents aggressive or unsafe control actions  
- Linearized MPC around hover offers a good trade-off between performance and complexity  

---

## Future Improvements
- Extension to full nonlinear MPC (NMPC)  
- Inclusion of wind and external disturbance models  
- Integration with real-time embedded hardware  
- Addition of obstacle-aware trajectory planning  

---

## Tools & Environment
- MATLAB / Simulink  
- Model Predictive Control Toolbox  
- State-space modeling and numerical optimization
