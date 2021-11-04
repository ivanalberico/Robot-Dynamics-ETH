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
% [ M ] = genmassmatrix(m, Theta, C, r, J_P, J_R)
%
% Computes the generalized mass matrix of a single body with scalar mass
% 'm' and inertial tensor 'Theta'. The latter is assumed to be computed
% using a local body-fixed frame which is offset w.r.t to the body's CoM by
% the vector 'r':
%
%   r_Ci = r_Bi + r
%
% The rotation matrix 'C' must be that which transforms the inertial tensor
% 'Theta' from the local body-fixed frame in which it is computed, to the
% frame in which the Jacobian matrices 'J_P' and 'J_R' are computed:
%
%   A_Theta = C * B_Theta * C', C = C_AB
%
% The compuation used accounts for the linear-rotational motioc coupling
% incurred by computing 'M' at a point which is not the body's CoM. If 'r'
% is a zero vector, however, then it is implied that 'J_P' and 'J_R' have
% been computed at the CoM.

function [ M ] = genmassmatrix(m, Theta, C, r, J_P, J_R)

    M = m*(J_P.'*J_P) +  J_R.'*(C*Theta*C.')*J_R - 2*m*(J_P.'*skew(C*r)*J_R);

end
