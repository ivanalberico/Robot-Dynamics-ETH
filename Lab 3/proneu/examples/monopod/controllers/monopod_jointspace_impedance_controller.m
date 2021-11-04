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
%   File:           monopod_jointspace_inverse_dynamics_controller.m
%
%   Authors:        Vassilios Tsounis, tsounisv@ethz.ch
%
%   Date:           1/2/2017
%
%   Desciption:     Joint-space impedance controller with feed-forward
%                   gravity compensation.
%

function [ T_j ] = monopod_jointspace_impedance_controller(model,world,logger,t,x)
    
    params = model.parameters.values;
    qdata = x(1:3);
    
    % Compute gravity compensation
    g_bar = model.dynamics.compute.g(qdata,[],[],[],params);
    S_bar = model.dynamics.compute.S_a(qdata,[],[],[],params);
    tau_a_g = S_bar.'\g_bar;
    
    % Set hip gains
    K_H_P = 50.0e+1;
    K_H_D = 20.0e+0;
    
    % Set knee gains
    K_K_P = 50.0e+1;
    K_K_D = 20.0e+0;
    
    % Get measurements
    q_H = x(2);
    q_K = x(3);
    dq_H = x(5);
    dq_K = x(6);
    
    % Set desired
    q_Hd = -pi/3;
    q_Kd = pi*2/3;
    dq_Hd = 0.0;
    dq_Kd = 0.0;
    
    % HIP Freeze Controller
    poserror_H = q_Hd - q_H;
    velerror_H = dq_Hd - dq_H;
    T_HFE = K_H_P*poserror_H + K_H_D*velerror_H;
    
    % KNEE Freeze Controller
    poserror_K = q_Kd - q_K;
    velerror_K = dq_Kd - dq_K;
    T_KFE = K_K_P*poserror_K + K_K_D*velerror_K;
    
    % Compose joint torques vector
    T_j = [T_HFE T_KFE].' + tau_a_g;
    
end
