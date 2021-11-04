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

function [ b ] = bodynonlinearforces(u, m_Bi, Bi_r_BiCi, Bi_Theta_Bi, C_IBi, I_omega_ICi, I_J_ICi, I_dJ_ICi, ns)
    
    % Transform the body inertia tensor
    Bi_Theta_Ci = Bi_Theta_Bi + m_Bi * skew(Bi_r_BiCi)^2;
    
    % Get Jacobian matrices
    I_J_P_ICi = I_J_ICi(1:3,:);
    I_J_R_ICi = I_J_ICi(4:6,:);
    
    % Get derivative Jacobian matrices
    I_dJ_P_ICi = I_dJ_ICi(1:3,:);
    I_dJ_R_ICi = I_dJ_ICi(4:6,:);
    
    % Transform to local Bi-Frame
    Bi_omega_ICi = C_IBi.' * I_omega_ICi;
    Bi_J_R_ICi = C_IBi.' * I_J_R_ICi;
    Bi_dJ_R_ICi = C_IBi.' * I_dJ_R_ICi;
    
    % Compute non-linear force contributions from linear and angular motion
    % of the body center of mass
    b_0 = m_Bi * I_J_P_ICi.' * (I_dJ_P_ICi * u);
    b_1 = Bi_J_R_ICi.' * Bi_Theta_Ci * (Bi_dJ_R_ICi * u);   
    b_2 = Bi_J_R_ICi.' * skew(Bi_omega_ICi) * (Bi_Theta_Ci * Bi_omega_ICi);
    
    % Compute total contribution of body motion of Bi to non-linear forces
    b = b_0 + b_1 + b_2;    
    
    % Perform element-wise implifications
    %b = simplesym(b, 'elementwise', ns);
    
end
