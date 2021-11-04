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
%
% [ F ] = getInverseRotationRateMapQuaternion(q)
% 
% The inverse rotation rate matrix F maps Cartesian angular velocities to
% derivatives of the orientation parametrization.
% 
% In the case of using unit quaternions as a choice of parametrization,
% there are two choices for the mapping matrix F, depending on which
% angular velocity we wish to use, or in fact, which is directly
% measurable. The two choices are:
% 
%   1. I_omega_IB: Absolute body angular velocity expressed in the inertial
%   frame.
% 
%   2. B_omega_IB: Absolute body angular velocity expressed in the inertial
%   local body fixed frame.
% 
% Thus, these two choices present us with the following two options for the
% mapping matrix F:
% 
% dp_IB = (1/2) * Hbar^T * B_omega_IB
%       = (1/2) * H^T    * I_omega_IB
% 
% Where,
% 
% H =
% [ -q1,  q0, -q3,  q2]
% [ -q2,  q3,  q0, -q1]
% [ -q3, -q2,  q1,  q0]
% 
% Hbar =
% [ -q1,  q0,  q3, -q2]
% [ -q2, -q3,  q0,  q1]
% [ -q3,  q2, -q1,  q0]
% 

function [ F_A, F_B ] = getInverseRotationRateMapQuaternion(q)
    
    % Compute F matrix as: (1/2)*H_bar.'
    F_A = 0.5*[-q(2), -q(3), -q(4);
                q(1),  q(4), -q(3);
                q(4),  q(1), -q(2);
                q(3), -q(2),  q(1)];

    % Compute F matrix as: (1/2)*H_bar.'
    F_B = 0.5*[-q(2), -q(3), -q(4);
                q(1), -q(4),  q(3);
                q(4),  q(1), -q(2);
               -q(3),  q(2),  q(1)];
end