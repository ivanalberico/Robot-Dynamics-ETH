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

%%
%   File:           monopod_jointspace_pid_controller.m
%
%   Authors:        Vassilios Tsounis, tsounisv@ethz.ch
%
%   Date:           1/2/2017
%
%   Desciption:     Joint-space PID motion controller.
%

function [ T_j ] = monopod_jointspace_pid_controller(model,world,logger,t,x)
    
    % Integrators
    persistent integrator_H;
    persistent integrator_K;

    % Set hip gains
    K_H_P = 5.0e+2/10;
    K_H_I = 1.0e+1/10;
    
    % Set knee gains
    K_K_P = 1.0e+3/10;
    K_K_I = 5.0e+1/10;
    
    % Get measurements
    dq_H = x(5);
    dq_K = x(6);
    
    % Reset integrators
    if t <= 0.05
        integrator_H = 0;
        integrator_K = 0;
    end
    
    % Set desired
    dq_Hd = 0.0;
    dq_Kd = 0.0;
    
    % HIP Freeze Controller
    velerror_H = dq_Hd - dq_H;
    integrator_H = integrator_H + K_H_I*velerror_H;
    T_HFE = K_H_P*velerror_H + integrator_H;
    
    % KNEE Freeze Controller
    velerror_K = dq_Kd - dq_K;
    integrator_K = integrator_K + K_K_I*velerror_K;
    T_KFE = K_K_P*velerror_K + integrator_K;
    
    % Compose joint torques vector
    T_j = [T_HFE T_KFE].';
    
end
