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

function [ obj ] = stlmesh(ax, filename, scale, r, C, color)

    % Generate data from the specified file
    [x,y,z] = cstlread(filename);
    c = ones(1,length(x));

    % Correct scaling
    x = x*scale;
    y = y*scale;
    z = z*scale;

    % Correct rotation and translation offsets
    Toff = eye(4);
    Toff(1:3,4) = r;
    Toff(1:3,1:3) = C;
    for i=1:length(x)
        r0 = [x(i) y(i) z(i) 1].';
        r1 = Toff*r0;
        x(i) = r1(1);
        y(i) = r1(2);
        z(i) = r1(3);
    end

    % Create and store patch data
    obj = cell(1,1);
    obj{1} = patch(x, y, z, c, 'visible','off', 'Parent', ax);
    obj{1}.EdgeColor = 'none';
    obj{1}.FaceColor = color;

end
