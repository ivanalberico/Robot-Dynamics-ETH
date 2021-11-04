function [ q_b, qd_b ] = base_motion_trajectory(model, t, x)
% Implements a motion planner which generates reference trajectories for
% the base of the robot.
% 
% Outputs:
%   q_b   : [3x1] Desired configuration of the base in the XZ-plane.
%   qd_b  : [3x1] Desired velocity of the base in the XZ-plane.
% 
% Inputs:
%   model : The Multi-body dynamics and kinematics model of the system.
%   t     : Current time [s]
%   x     : Current state of the system (position, velocity)
% 

%% Trajectory Generation

% Defines circular trajectory parameters
offset = [0, 0.45, 0]';
amplitude = 0.15;
freq = 1.0;

% Base X-Z-Theta references
q_b = amplitude.*[sin(2*pi*freq*t), cos(2*pi*freq*t), 0].' + offset;
qd_b = amplitude.*freq.*[cos(2*pi*freq*t), -sin(2*pi*freq*t), 0].';

end

%% EOF