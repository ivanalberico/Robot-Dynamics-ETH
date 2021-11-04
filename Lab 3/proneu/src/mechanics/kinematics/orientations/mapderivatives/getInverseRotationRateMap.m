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
% F = getInverseRotationRateMap( orientation, phi )
%
% Computes the matrix 'F' for conversion of the angular velocity 'omega' 
% (i.e. quasi-velocity) to the vector of parametrization velocities 
% dphi/dt. 'F' is then a function of the vector of orientation 
% parametrization 'phi':
%
% dphi/dt = F(phi) * omega
%
% The resulting expressions are dependent on the parametrization used to
% represent the orientation of the base w.r.t the inertial frame via the 
% 'orientation' argument, which is also used to define elements of 'phi'.

function [ F_A, F_B ]  = getInverseRotationRateMap( orientation, phi )
    switch orientation
        case 'quaternion'
            [F_A, F_B] = getInverseRotationRateMapQuaternion(phi);
        case 'angleaxis'
            [F_A, F_B] = getInverseRotationRateMapAngleAxis(phi(1), phi(2:4));
        case 'taitbryanzyx'
            [F_A, F_B] = getInverseRotationRateMapTaitBryanZyx(phi);
        case 'cardanxyz'
            [F_A, F_B] = getInverseRotationRateMapCardanXyz(phi);
        case 'eulerzyz'
            [F_A, F_B] = getInverseRotationRateMapEulerZyz(phi);
        case 'eulerzxz'
            [F_A, F_B] = getInverseRotationRateMapEulerZxz(phi);
        otherwise
            error('The type of parametrization of orientation was not specified.');
    end
end
