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

function [ obj ] = genellipsoid(ax, a, b, c, color, alpha, offsets)

    % Create the cylindrical shaft
    [X,Y,Z] = ellipsoid(0,0,0,a,b,c);
    C = zeros(size(X,1),size(X,2));

    % Create the position and orientation offsets
    Toffset = eye(4);
    Toffset(1:3,4) = -offsets.r;
    Toffset(1:3,1:3) = offsets.C;

    % Initialzie to desired pose
    for i=1:numel(X)
        r_0 = [X(i) Y(i) Z(i) 1].';
        r_1 = Toffset*r_0;
        X(i) = r_1(1);
        Y(i) = r_1(2);
        Z(i) = r_1(3);
    end

    % Creat the sphere object
    obj = cmesh(X, Y, Z, C, 'Parent', ax);
    obj.EdgeColor = 'none';
    obj.FaceColor = color;
    obj.FaceAlpha = alpha;
    obj.FaceLighting = 'gouraud';

end
