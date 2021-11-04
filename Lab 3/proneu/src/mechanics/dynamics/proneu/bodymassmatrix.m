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

function [ M ] = bodymassmatrix(m_Bi, Bi_r_BiCi, Bi_Theta_Bi, C_IBi, I_J_ICi, ns)

    % Transform the body inertia tensor
    Bi_Theta_Ci = Bi_Theta_Bi + m_Bi * skew(Bi_r_BiCi)^2;
    
    % Get Jacobian matrices
    I_J_P_ICi = I_J_ICi(1:3,:);
    I_J_R_ICi = I_J_ICi(4:6,:);
    Bi_J_R_ICi = C_IBi.' * I_J_R_ICi;
    
    % The translational kinetic energy is computed via the absolute linear
    % velocity of the local Ci-Frame expressed in the same
    M_P = m_Bi * (I_J_P_ICi.' * I_J_P_ICi);
    
    % The rotational kinetic energy is computed via the absolute angular
    % velocity of the local Ci-Frame, expressed in the body frame Bi
    M_R = Bi_J_R_ICi.' * (Bi_Theta_Ci) * Bi_J_R_ICi;
    
    % Sum the total generalized mass matrix contribution of body
    M = M_P + M_R;
    
    % Perform element-wise implifications
    M = simplesym(M, 'elementwise', ns);
    
end

