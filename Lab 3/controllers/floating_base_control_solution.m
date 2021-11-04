function [ tau ] = floating_base_control_solution(model, t, x)
% Implements a hierarchical-QP-based controller which realizes a desired 
% base and end-effector motion.
% 
% Outputs:
%   tau   : [7x1] torques [tau_F, tau_H, tau_A]'
% 
% Inputs:
%   model : The Multi-body dynamics and kinematics model of the system.
%   t     : Current time [s]
%   x     : Current state of the system (position, velocity)
% 
% Where:
%	tau_F : [2x1] torques [hip, knee]' (front leg)
%	tau_H : [2X1] torques [hip, knee]' (hind leg)
%	tau_A : [3x1] torques [shoulder, elbow, wrist]' (arm)
% 

%% System State

% Extract generalized positions and velocities
q = x(1:10);    % [10x1] Generalized coordinates [q_b, q_F, q_H, q_A]'
dq = x(11:20);  % [10X1] Generalized velocities

%% Model Parameters 

% Extract dynamics at current state
params = model.parameters.values;
M = model.dynamics.compute.M(q,dq,[],[],params); % [10X10] Inertia matrix
b = model.dynamics.compute.b(q,dq,[],[],params); % [10X1] Nonlinear-dynamics vector
g = model.dynamics.compute.g(q,dq,[],[],params); % [10X1] Gravity vector
S = [zeros(7, 3), eye(7)];                       % [7x10] Selection matrix

% Get Jacobians and Derivatives at current state
J_B  = eval_jac(model.body( 1).kinematics.compute.I_J_IBi, q, [], params);    % [3X10] body position and orientation Jacobian
J_FF = eval_jac(model.body( 7).kinematics.compute.I_J_IBi, q, [], params);	% [3X10] Front foot position and orientation Jacobian
J_HF = eval_jac(model.body( 4).kinematics.compute.I_J_IBi, q, [], params);	% [3x10] Hind foot position and orientation Jacobian
J_EE = eval_jac(model.body(11).kinematics.compute.I_J_IBi, q, [], params);	% [3x10] Arm End-effector position and orientation Jacobian
dJ_B  = eval_jac(model.body( 1).kinematics.compute.I_dJ_IBi, q, dq, params);  % [3X10] body position and orientation Jacobian derivative
dJ_FF = eval_jac(model.body( 7).kinematics.compute.I_dJ_IBi, q, dq, params);  % [3X10] Front foot position and orientation Jacobian derivative
dJ_HF = eval_jac(model.body( 4).kinematics.compute.I_dJ_IBi, q, dq, params);  % [3x10] Hind foot position and orientation Jacobian derivative
dJ_EE = eval_jac(model.body(11).kinematics.compute.I_dJ_IBi, q, dq, params);  % [3x10] Arm End-effector position and orientation Jacobian derivative

% Extract forward kinematics of the base
p_B = q(1:3);
w_B = dq(1:3);

% Extract forward kinematics of the end-effector
T_I_EE = model.body(11).kinematics.compute.T_IBi(q, [], [], params); % [4x4] Homogeneous transfrom from inertial to EE frame
p_EE = [T_I_EE(1,4); T_I_EE(3,4); acos(T_I_EE(1,1))]; % [3x1] pose of the EE
w_EE = J_EE(1:3,:)*dq; % [3x1] xz twist of the EE

% Assemble constraint Jacobian -> Only constrain linear velocity at feet
J_c = [J_FF(1:2,:) ; J_HF(1:2,:)];
dJ_c = [dJ_FF(1:2,:); dJ_HF(1:2,:)];

% Reduce end-effector Jacobian to only contain linear velocity terms
J_EE = J_EE(1:3,:);
dJ_EE = dJ_EE(1:3,:);

%% Control References

% Query the base trajectory planner
[p_star_B, w_star_B] = base_motion_trajectory(model, t, x);

% End-effector references
p_star_EE = [0.7 0.8 1.8].';
w_star_EE = zeros(3,1);

%% Optimization Tasks
% x_opt = [ddq', f_c_xz', tau']

% Equations of motions
A_eom = [M, -J_c', -S'];
b_eom = -b -g;%dim=10

% No foot-contact motions
A_c = [J_c, zeros(4, 4), zeros(4, 7)];
b_c = -dJ_c*dq;%dim=4

% Body motion
kp_b = 10;
kd_b = 2*sqrt(kp_b);
dw_star_B = kp_b*(p_star_B - p_B) + kd_b*(w_star_B - w_B);
A_b = [J_B, zeros(3, 4), zeros(3, 7)];
b_b = dw_star_B - dJ_B*dq;%dim=3

% No end-effector motion
kp_ee = 5;
kd_ee = 2*sqrt(kp_ee);
wd_ee = kp_ee*(p_star_EE - p_EE) + kd_ee*(w_star_EE - w_EE);
A_ee = [J_EE, zeros(size(J_EE,1), 4), zeros(size(J_EE,1), 7)];
b_ee = wd_ee-dJ_EE*dq;%dim=3

%% Additional Tasks

% Kinematic null space position control
q0 = [0 0.50 0 0.9 -1.5 0.9 -1.5 0.9 0.7 0.4]'; % [10x1] Default generalized coordinates
kp_0 = 1;
kd_0 = 2*sqrt(kp_0);
A_0 = [eye(10), zeros(10, 4), zeros(10, 7)];
b_0 = kp_0*(q0 - q) - kd_0*dq;%dim=10

% Torque minimization
A_tau = [zeros(7, 10), zeros(7, 4), eye(7)];
b_tau = zeros(7,1);%dim=7

% Contact force minimization
A_f_c = [zeros(4, 10), eye(4), zeros(4, 7)];
b_f_c = zeros(4,1);%dim=4

%% Inequality Constraints

% Torque limits
tau_max = 50;
C_tau_up = [zeros(7, 10), zeros(7, 4), eye(7)];
d_tau_up = tau_max*ones(7,1);
C_tau_low = -C_tau_up;%dim={0,..,7}
d_tau_low = d_tau_up;%dim={0,..,7}

% Friction Cone
mu = 0.5;
C_cone = [0 -1; 1 -mu; -1 -mu];
C_f_c = [zeros(6, 10), blkdiag(C_cone, C_cone), zeros(6, 7)];
d_f_c = zeros(6, 1);%dim={0, 1, 2}

%% Assemble and solve

% Define hierarchy of tasks
A = {A_eom, A_c, A_b, A_ee, A_0, A_tau, A_f_c};
b = {b_eom, b_c, b_b, b_ee, b_0, b_tau, b_f_c};

% Define inequality constraints
C = [C_tau_up; C_tau_low; C_f_c];
d = [d_tau_up; d_tau_low; d_f_c];

% Solve hierarchical QPs
print_solution = 0;
x_opt = hopt(A, b, C, d, print_solution);
ddq = x_opt(1:10);
f_c = x_opt(11:14);
tau = x_opt(15:end);

end

%% Helper functions

function jac = eval_jac(f, q, dq, params)
jac = f(q,dq,[],params);
jac = jac(1:2:end,:);
end

%% EOF
