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
%   Desciption:     Joint-space inversed dynamics controller for a planar
%                   monopod hopper.
%

function [ T_j ] = monopod_jointspace_inverse_dynamics_controller(model,world,logger,t,x)
    
    % Integrators
    persistent integrator_H;
    persistent integrator_K;
    persistent interpol_counter;
    persistent q_Ht;
    persistent q_Kt;
    persistent dq_Ht;
    persistent dq_Kt;
    
    % Reset integrators
    if t <= 0.05
        integrator_H = 0;
        integrator_K = 0;
        interpol_counter = 0;
    end
    
    params = model.parameters.values;
    qdata = x(1:3);
    dqdata = x(4:6);
    
    % Dynamic parameters
    M_bar = model.dynamics.compute.M(qdata,dqdata,[],[],params);
    b_bar = model.dynamics.compute.b(qdata,dqdata,[],[],params);
    g_bar = model.dynamics.compute.g(qdata,dqdata,[],[],params);
    S_bar = model.dynamics.compute.S_a(qdata,[],[],[],params);
    
    J_c = model.body(4).kinematics.compute.I_J_IBi(qdata,[],[],params);
    T_c = model.body(4).kinematics.compute.T_IBi(qdata,[],[],params);
    g_c_N = T_c(3,4) - 0.0;
    
    inv_M_bar = eye(3)\M_bar;
    N_c_bar = eye(3) - inv_M_bar * J_c.' * (J_c * inv_M_bar * J_c.') * J_c;
    X_bar = eye(3)\(N_c_bar.' * S_bar.');
    X_bar = X_bar.' * N_c_bar.';
    
    % Set hip gains
    K_H_P = 1.0e+4;
    K_H_D = 1.0e+2;
    K_H_I = 1.0e-2;
    
    % Set knee gains
    K_K_P = 1.0e+4;
    K_K_D = 1.0e+2;
    K_K_I = 1.0e-2;
    
    % Set measurements
    q_H = qdata(2);
    q_K = qdata(3);
    dq_H = dqdata(2);
    dq_K = dqdata(3);
    
    % Set desired
    
    if t>=2.0 && t<= 2.17
        % Jump mode
        interpol_counter = interpol_counter + 1;
        if interpol_counter == 1
            Njstp = length(2.0:1e-4:2.17);
            q_Ht = linspace(q_H,    -pi*1/10, Njstp);
            q_Kt = linspace(q_K,    pi*2/10, Njstp);
            dq_Ht = linspace(dq_H,  0.1, Njstp);
            dq_Kt = linspace(dq_K,  -0.1, Njstp);
        end
        q_Hd = q_Ht(interpol_counter);
        q_Kd = q_Kt(interpol_counter);
        dq_Hd = dq_Ht(interpol_counter);
        dq_Kd = dq_Kt(interpol_counter);
        
    else
        % Stand mode
        q_Hd = -pi/3;
        q_Kd = pi*2/3;
        dq_Hd = 0.0;
        dq_Kd = 0.0;
    end
    
    % Get motion tracking errors
    poserror_H = q_Hd - q_H;
    velerror_H = dq_Hd - dq_H;
    poserror_K = q_Kd - q_K;
    velerror_K = dq_Kd - dq_K;
    
    % Set inverse dynamics controller output
    ddq_Hd = K_H_P*poserror_H + K_H_D*velerror_H;
    ddq_Kd = K_K_P*poserror_K + K_K_D*velerror_K;
    ddq_desired = 0.00001*[0 ddq_Hd ddq_Kd].';
    tau_DCNSPIDC = X_bar*(M_bar * ddq_desired + b_bar + g_bar);
    
    if g_c_N<=0.0
        T_j = tau_DCNSPIDC;
        integrator_H = 0;
        integrator_K = 0;
    else
        integrator_H = integrator_H + K_H_I*poserror_H;
        integrator_K = integrator_K + K_K_I*poserror_K;
        T_HFE = K_H_P*poserror_H + K_H_D*velerror_H + integrator_H;
        T_KFE = K_K_P*poserror_K + K_K_D*velerror_K + integrator_K;
        T_j = [T_HFE T_KFE].';
    end
    
end

%%
% EOF