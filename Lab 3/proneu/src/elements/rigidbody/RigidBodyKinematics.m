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

classdef RigidBodyKinematics
    
    % Defining quantities
    properties
        % Body-CoM to Body-frame configuration
        T_BiCi
        % Parent-to-body configuration
        T_PiBi
    end
    
    % Derived quantities
    properties
        
        %%% Position & Orientation Kinematics
        
        % Base-to-body relative configuration
        T_BBi
        T_BCi
        
        % Absolute body configuration
        T_IBi
        T_ICi
        
        %%% Velocity Kinematics
        
        % Base-to-Body velocity kinematics
        B_v_BBi
        B_omega_BBi
        B_J_BBi
        I_v_BBi
        I_omega_BBi
        I_J_BBi
        
        % Base-to-Body CoM velocity kinematics
        B_v_BCi
        B_omega_BCi
        B_J_BCi
        I_v_BCi
        I_omega_BCi
        I_J_BCi
        
        % Inertial-to-Body velocity kinematics
        B_v_IBi
        B_omega_IBi
        B_J_IBi
        I_v_IBi
        I_omega_IBi
        I_J_IBi
        
        % Inertial-to-Body CoM velocity kinematics
        B_v_ICi
        B_omega_ICi
        B_J_ICi
        I_v_ICi
        I_omega_ICi
        I_J_ICi
        
        %%% Acceleration Kinematics
        
        % Base-to-Body acceleration kinematics
        B_a_BBi
        B_psi_BBi
        B_dJ_BBi
        I_a_BBi
        I_psi_BBi
        I_dJ_BBi
        
        % Base-to-Body CoM acceleration kinematics
        B_a_BCi
        B_psi_BCi
        B_dJ_BCi
        I_a_BCi
        I_psi_BCi
        I_dJ_BCi
        
        % Inertial-to-Body acceleration kinematics
        B_a_IBi
        B_psi_IBi
        B_dJ_IBi
        I_a_IBi
        I_psi_IBi
        I_dJ_IBi
        
        % Inertial-to-Body CoM acceleration kinematics
        B_a_ICi
        B_psi_ICi
        B_dJ_ICi
        I_a_ICi
        I_psi_ICi
        I_dJ_ICi
    end
end
