% proNEu: A tool for rigid multi-body mechanics in robotics.
% 
% Copyright (C) 2017  Marco Hutter, Christian Gehring, C. Dario Bellicoso,
% Vassilios Tsounis
% 
% This program is free software: you can redistribute it and/or modify
% it under the terms of the GNU General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% any later version.
% 
% This program is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU General Public License for more details.
% 
% You should have received a copy of the GNU General Public License
% along with this program.  If not, see <http://www.gnu.org/licenses/>.
%
%   File:           planarspotmini_simulation.m
%
%   Authors:        Vassilios Tsounis, tsounisv@ethz.ch
%
%   Date:           3/11/2017
%
%   Desciption:     A simple controller for an under-actuated double
%                   pendulum cart-pole system.
%

function [ tau ] = planarspotmini_controller(model,world,logger, t, x)
    
    % ARGUMETNS:
    %
    % model: the robot model object
    % world: the world model object
    % logger: data logger instance <ingore this argument>
    % t: The current time
    % x: The state of the robot, x := [q_B; u_B; q_j; dq_j]

    % Base configuration and velocities
    % --> These would be computed by a state-estimator from IMU data
    q_b = x(1:3);
    dq_b = x(11:13);
   
    % Joint configurations and velocities
    % --> These would be computed by a state-estimator from actuator data
    q_j = x(4:10);
    dq_j = x(14:20);

    % DEMO JOINT-FREEZE CONTROLLER
    % Desired nominal joint configurations
    q_j_star = [0.9 -1.5 0.9 -1.7 0.9 0.7 0.4].';
    dq_j_star = zeros(7,1);
    tau = zeros(7,1);
    for i=1:7
       tau(i) = 150.0*(q_j_star(i) - q_j(i)) + 2.0*(dq_j_star(i) - dq_j(i));
    end

end