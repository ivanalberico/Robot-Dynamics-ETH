function [ tau ] = control_op_space_hybrid_solution( I_r_IE_des, eul_IE_des, q, dq, I_F_E_x )
% CONTROL_OP_SPACE_HYBRID Operational-space inverse dynamics controller 
% with a PD stabilizing feedback term and a desired end-effector force.
%
% I_r_IE_des --> a vector in R^3 which describes the desired position of the
%   end-effector w.r.t. the inertial frame expressed in the inertial frame.
% eul_IE_des --> a set of Euler Angles XYZ which describe the desired
%   end-effector orientation w.r.t. the inertial frame.
% q --> a vector in R^n of measured joint positions
% q_dot --> a vector in R^n of measured joint velocities
% I_F_E_x --> a scalar value which describes a desired force in the x
%   direction

% Design the control gains
kp = 50.0;
kd = 14.0;
kpMat = kp * diag([1.0 1.0 1.0 1.0 1.0 1.0]);
kdMat = kd * diag([1.0 1.0 1.0 1.0 1.0 1.0]);

% Desired end-effector force
I_F_E = [I_F_E_x, 0.0, 0.0, 0.0, 0.0, 0.0]';

% Find jacobians, positions and orientation
I_Je = I_Je_fun_solution(q);
I_dJ_e = I_dJe_fun_solution(q, dq);
T_IE = T_IE_fun_solution(q);
I_r_IE = T_IE(1:3, 4);
C_IE = T_IE(1:3, 1:3);

% Define error orientation using the rotational vector parameterization.
C_IE_des = eulAngXyzToRotMat(eul_IE_des);
C_err = C_IE_des*C_IE';
orientation_error = rotMatToRotVec_solution(C_err);

% Define the pose error.
chi_err = [I_r_IE_des - I_r_IE;
           orientation_error];

% Project the joint-space dynamics to operational space
M = M_fun_solution(q);
b = b_fun_solution(q, dq);
g = g_fun_solution(q);
j_invm = I_Je/M;
lambda = pseudoInverseMat_solution(j_invm*I_Je', 0.01);
mu = lambda*(j_invm*b - I_dJ_e*dq);
p =  lambda*j_invm*g;

% Define the motion and force selection matrices.
Sm = eye(6);
Sm(1,1) = 0;
Sf = eye(6)-Sm;

% Design a controller which implements the operational-space inverse
% dynamics and exerts a desired force.
dchi_err = zeros(6,1) - I_Je * dq; % target velocity is zero.
I_F_E_des = lambda * Sm * (kpMat*chi_err + kdMat*dchi_err) ...
          + Sf*I_F_E + mu + p;

% Map the desired force back to the joint-space torques
tau = I_Je'*I_F_E_des;

end
