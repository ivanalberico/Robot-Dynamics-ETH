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

function C = mapAngleAxisToRotationMatrix(theta, n)
% MAPANGLEAXISTOROTATIONMATRIX maps an angle-axis parametrization to SO(3)
% The function MAPANGLEAXISTOROTATIONMATRIX(theta, n) maps an angle-axis 
% phi = [theta n_x n_y n_z]' to a rotation matrix C.
%
% Author(s): Dario Bellicoso, Vassilios Tsounis

% Assure n is 3x1 array
n = n(:);

% Compute skew-symmetric and tensor products
Sn = skew(n);
Xn = n*n.';

% Apply the Rodrigeus rotation formula
C = cos(theta)*eye(3) + sin(theta)*Sn + (1-cos(theta))*Xn;

% Simplify if we are computing a symbolic expression
if isa(n, 'sym')
    C = simplify(C);
end

end
