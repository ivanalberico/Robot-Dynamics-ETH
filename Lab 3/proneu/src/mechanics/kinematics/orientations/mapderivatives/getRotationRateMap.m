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
% F = getRotationRateMap( orientation, phi )
%
% Computes the matrix 'invF' for computing the angular velocity 'omega' 
% (i.e. quasi-velocity) from the vector of parametrization velocities 
% dphi/dt. 'invF' is then a function of the vector of orientation 
% parametrization 'phi':
%
% omega = G(phi) * dphi/dt
%
% The resulting expressions are dependent on the parametrization used to
% represent the orientation of the base w.r.t the inertial frame via the 
% 'orientation' argument, which is also used to define elements of 'phi'.
% 

function [ G_A,G_B ] = getRotationRateMap( orientation, phi )
    switch orientation
        case 'quaternion'
            [G_A,G_B] = getRotationRateMapQuaternion(phi);
        case 'angleaxis'
            [G_A,G_B]  = getRotationRateMapAngleAxis(phi(1), phi(2:4));
        case 'taitbryanzyx'
            [G_A,G_B]  = getRotationRateMapTaitBryanZyx(phi);
        case 'cardanxyz'
            [G_A,G_B]  = getRotationRateMapCardanXyz(phi);
        case 'eulerzyz'
            [G_A,G_B]  = getRotationRateMapEulerZyz(phi);
        case 'eulerzxz'
            [G_A,G_B]  = getRotationRateMapEulerZxz(phi);
        otherwise
            error('The type of parametrization of orientation was not specified.');
    end
end
