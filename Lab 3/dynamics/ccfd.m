function [ddq, F_C] = ccfd(model, x, u, F_EE)
% Implements Constraint-Consistent Forward Dynamics of a simplified 
% planar Spot-Mini robotic system.
% 
% Outputs:
%   ddq   : [10x1] Vector of joint accelerations.
%   F_C   : [6x1] Vector of constraint forces acting at the feet.
% 
% Inputs:
%   model : The Multi-body dynamics and kinematics model of the system.
%   x     : [20x1] State vector of the system (positions, velocities).
%   u     : [7x1] Input vector of the system [tau_F, tau_H, tau_A]' .
%   F_EE  : [3x1] 2D force acting on the end-effector from the environment.
% 

% TODO
persistent I_p_Ff0 I_p_Hf0;

% Map system quantities to multi-body physical quantities
q = x(1:10);    % [10x1] Generalized coordinates [q_b, q_F, q_H, q_A]'
dq = x(11:20);  % [10X1] Generalized velocities
tau = u;        % [17X1] Control input mapping directly to joint torques

% Extract dynamical parameters at current state
params = model.parameters.values;
M_d = model.dynamics.compute.M(q,dq,[],[],params); % [10X10] Inertia matrix
b_d = model.dynamics.compute.b(q,dq,[],[],params); % [10X1] Nonlinear forces
g_d = model.dynamics.compute.g(q,dq,[],[],params); % [10X1] Gravity forces

% Compute Jacobians and respective derivatives at current state
I_J_Ff = eval_jac(model.body( 7).kinematics.compute.I_J_IBi, q, [], params);	% [3X10] Front foot position and orientation Jacobian
I_J_Hf = eval_jac(model.body( 4).kinematics.compute.I_J_IBi, q, [], params);	% [3x10] Hind foot position and orientation Jacobian
I_J_EE = eval_jac(model.body(11).kinematics.compute.I_J_IBi, q, [], params);	% [3x10] Arm End-effector position and orientation Jacobian
I_Jd_Ff = eval_jac(model.body( 7).kinematics.compute.I_dJ_IBi, q, dq, params);  % [3X10] Front foot position and orientation Jacobian derivative
I_Jd_Hf = eval_jac(model.body( 4).kinematics.compute.I_dJ_IBi, q, dq, params);  % [3x10] Hind foot position and orientation Jacobian derivative

% Compute forward kinematics at current state
I_T_Ff = model.body(7).kinematics.compute.T_IBi(q, dq, [], params);
I_p_Ff = I_T_Ff([1,3], 4);
I_T_Hf = model.body(4).kinematics.compute.T_IBi(q, dq, [], params);
I_p_Hf = I_T_Hf([1,3], 4);

% TODO
if isempty(I_p_Ff0)
    I_p_Ff0 = I_p_Ff;
    I_p_Hf0 = I_p_Hf;
end

% Assemble foot constraints -> no linear velocity
I_J_c = [I_J_Ff(1:2,:) ; I_J_Hf(1:2,:)];
I_Jd_c = [I_Jd_Ff(1:2,:); I_Jd_Hf(1:2,:)];
num_constr = size(I_J_c,1);

% Selection Matrix
S = [zeros(7, 3), eye(7)]; 

% Drift correction
kp = 10;
kd = 2*sqrt(kp);
wd_drift = [kp*(I_p_Ff0 - I_p_Ff); kp*(I_p_Hf0 - I_p_Hf)] - kd*I_J_c*dq;

% Define matrix and vector of the linear system used to solve the dynamics
A = [M_d,    -I_J_c'; ...
     -I_J_c, zeros(num_constr)];
b = [S'*tau + I_J_EE'*F_EE - b_d - g_d; ...
     I_Jd_c * dq - wd_drift];
 
% Solve for acceleration and constraint force simultaniously
xd = A \ b; 
ddq = xd(1:10);
F_C = xd(11:14);

end

function jac = eval_jac(f, q, dq, params)
jac = f(q,dq,[],params);
jac = jac(1:2:end,:);
end

