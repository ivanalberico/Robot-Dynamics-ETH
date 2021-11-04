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

function [ tau ] = slip_raibert_controller(model,world,logger,t,x)
    
    %% System Data
    
    % Parameters and state
    params = model.parameters.values;
    qdata = x(1:8);
    dqdata = x(9:15);

    % Extract kinematic information of the foot and body.
    T_F = model.body(4).kinematics.compute.T_IBi(qdata,[],[],params);
    T_B = model.body(1).kinematics.compute.T_IBi(qdata,[],[],params);
    I_J_IB = model.body(1).kinematics.compute.I_J_IBi(qdata,[],[],params);
    C_IB = T_B(1:3,1:3);
    
    %% Spring Model
    
    % spring deflection
    zeta_S = qdata(8);
    dzeta_S = dqdata(7);
     
    % parameters
    Ks = 100.0;
    Ds = 0.1;
    zeta_0 = 0.3;
    
    % Compute total spring force
    F_Sbf = Ks * (zeta_S - zeta_0) + Ds*dzeta_S;
    
    %% Controller torques 
    
    % Angle of attack references (in Deg.)
    if (t <= 0.5)
        alpha_d = 0.0;
        beta_d = (1/360) * (2*pi);
    else
        alpha_d = 0.0;
        beta_d = (-5/360) * (2*pi);
    end
    
    % Attitude controller gain
    Kpr = -10.0;
    Kdr =  -1.0;
    
    % Compute control torques based on the relative orientation error
    C_IBd = mapEulerAnglesXyzToRotationMatrix([alpha_d; beta_d; 0]);
    C_err = C_IB*(C_IBd.');    
    delta_phi = unskew(logm(C_err));
    T_Bc = Kpr*delta_phi + Kdr*I_J_IB(4:6,:)*dqdata;
    
    % Constants
    z_Floor = 0;
    
    % Get foot coordinates
    z_Fk = T_F(3,4);
    
    % Normal gap function for contact detection
    g_N_check = z_Fk - z_Floor;
    
    % Check for closed contacts
    if g_N_check > 0
        % Control the angle of attack
        T_Bx = T_Bc(1);
        T_By = T_Bc(2);
    else
        T_Bx = 0;
        T_By = 0;
    end
    
    %% Total internal forces
    
    tau = [F_Sbf T_Bx T_By].';
    
end
