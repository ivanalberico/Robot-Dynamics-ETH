function [ tau ] = jointspace_pid_control(model, t, x)
% Implements a trivial joint-space PID controller on zero velocity references.
% 
% Outputs:
% 	tau : [7x1] torques [tau_F, tau_H, tau_A]'
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
q = x(1:10); % [10x1] Generalized coordinates [q_b, q_F, q_H, q_A]'
dq = x(11:20); % [10X1] Generalized velocities

%% Model Parameters 

% Extract model parameters 
params = model.parameters.values;

% Extract dynamics at current state
g = model.dynamics.compute.g(q,dq,[],[],params); % [10X1] Gravity vector
S = [zeros(7, 3), eye(7)];                       % [10X7] Selection matrix

%% Zero-Velocity Joint-Space PID Controller

% Joint configurations and velocities
q_j = q(4:10);
dq_j = dq(4:10);

% Reference joint configurations and velocities
q_star_j = [0.9  -1.5  0.9  -1.5  0.5  0.8  0.6].';
dq_star_j = zeros(7,1);

% PD Gains
Kp = [150.0*ones(1,4) 20.0*ones(1,3)].';
Kd = [2.0*ones(1,4) 0.2*ones(1,3)].';

% Joint torque command
tau = Kp .* (q_star_j - q_j) + Kd .* (dq_star_j - dq_j);

% Gravity compensation
tau = tau + S*g;

end

%% EOF