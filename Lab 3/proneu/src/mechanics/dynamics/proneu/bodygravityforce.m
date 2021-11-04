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

function [ g ] = bodygravityforce(m_Bi, I_J_ICi, I_a_g, ns)

    % Get position Jacobian matrix
    I_J_P_ICi = I_J_ICi(1:3,:);
    
    % Compute gravity force contributions from the motion of the
    % Center-of-Mass point
    g = I_J_P_ICi.' * (m_Bi * I_a_g);
    
    % Perform element-wise implifications
    g = simplesym(g, 'elementwise', ns);
    
end
