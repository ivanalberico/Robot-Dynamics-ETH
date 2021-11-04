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

function C = mapEulerAnglesZyxToRotationMatrix(phi)
% MAPEULERANGLESZYXTOROTATIONMATRIX(angles) maps a set of Euler angles to a
% rotation matrix in SO(3). The Euler angles represent a set successive
% rotations around Z-Y'-X''. This is equivalent to rotating around the
% fixed axes in X-Y-Z order.
%
% Author(s): Dario Bellicoso, Vassilios Tsounis

z = phi(1);
y = phi(2);
x = phi(3);

C = getRotationMatrixZ(z)*getRotationMatrixY(y)*getRotationMatrixX(x);

if isa(phi, 'sym')
    C = simplify(C);
end

end
