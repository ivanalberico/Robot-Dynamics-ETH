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

function E = getMapEulAngZYXDDiffToAngAccInWorldFrame(angles, dAngles)
% GETMAPEULANGZYXDDIFFTOANGACCINWORLDFRAME(angles, dAngles) returns the 3x3
% matrix that maps the time derivative of ZYX Euler angles to angular
% acceleration in world frame.
%
% Author(s): Dario Bellicoso

z = angles(1);
y = angles(2);

dz = dAngles(1);
dy = dAngles(2);

% The mapping is computed as:
% [wx]                 [dz]
% [wy] = E([z,y,x]') * [dy]
% [wz]                 [dx]
%
% The time derivative is then obtained by differentiating this mapping
% w.r.t. time.
E = [0      -dz*cos(z)      -dy*cos(z)*sin(y)-dz*cos(y)*sin(z);
     0      -dz*sin(z)       dz*cos(y)*cos(z)-dy*sin(y)*sin(z);
     0       0              -dy*cos(y)];


if (isa(angles, 'sym'))
    E = simplify(E);
end

end
