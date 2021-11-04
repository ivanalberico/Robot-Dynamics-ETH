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

function [ B_Theta_B ] = steiner(m, A_Theta_A, A_r_AB, C_BA)
    
    % Apply the Parallel Axis Theorm (PAT) for the translational transformation
    A_Theta_B = A_Theta_A - m * skew(A_r_AB)^2;
    
    % Apply the rotational transformation
    B_Theta_B = C_BA * ( A_Theta_B ) * C_BA.';
    
    % Simplify expression if symbolic
    if isa(B_Theta_B, 'sym')
        B_Theta_B = simplify(B_Theta_B);
    end

end
