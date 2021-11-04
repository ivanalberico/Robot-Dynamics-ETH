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

function phi = mapRotationMatrixToRotationVector(C)
% MAPROTATIONVECTORTOROTATIONMATRIX maps a rotation vector phi in so(3) to 
% a rotation matrix in SO(3)
%
%  syntax: phi = mapRotationMatrixToRotationVector(C)
%
% Author(s): Vassilios Tsounis

theta = acos((C(1,1)+C(2,2)+C(3,3)-1)/2);
n = [C(3,2)-C(2,3); C(1,3)-C(3,1); C(2,1)-C(1,2)] * (1/(2*sin(theta)));
phi = theta * n;

end
