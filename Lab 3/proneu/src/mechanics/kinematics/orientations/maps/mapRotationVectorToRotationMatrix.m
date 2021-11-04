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

function C = mapRotationVectorToRotationMatrix(phi)
% MAPROTATIONVECTORTOROTATIONMATRIX maps a rotation vector phi in so(3) to 
% a rotation matrix in SO(3)
%
%  syntax: C = mapRotationVectorToRotationMatrix(phi)
%
% Author(s): Dario Bellicoso

% phi is a rotation vector which describes the rotation of a frame w.r.t.
% another one. The rotation matrix in SO(3) can be computed by using the
% exponential map from so(3) to SO(3)
theta = norm(phi);

if (abs(theta) < eps)
    C = eye(3)   +   skew(phi);
else
    C = eye(3)   +   sin(theta)/theta*skew(phi)   +   (1-cos(theta))/(theta^2)*skew(phi)^2;
end

end
