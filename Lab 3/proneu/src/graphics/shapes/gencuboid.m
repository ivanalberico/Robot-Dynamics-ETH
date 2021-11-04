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

function [ obj ] = gencuboid(ax, l, w, h, color, alpha, offsets)

    % Create the verticies for the parrallelogram sides
    x=([0 1 1 0 0 0;1 1 0 0 1 1;1 1 0 0 1 1;0 1 1 0 0 0]-0.5)*l+l/2;
    y=([0 0 1 1 0 0;0 1 1 0 0 0;0 1 1 0 1 1;0 0 1 1 1 1]-0.5)*w+w/2;
    z=([0 0 0 0 0 1;0 0 0 0 0 1;1 1 1 1 0 1;1 1 1 1 0 1]-0.5)*h+h/2;
    c = zeros(size(x));

    % Create the position and orientation offsets
    r_off = -offsets.r;
    r_off = r_off(:);
    r_off = r_off - [l/2 w/2 h/2].';
    Toffset = eye(4);
    Toffset(1:3,4) = r_off;
    Toffset(1:3,1:3) = offsets.C;

    % Initialzie to desired pose
    for i=1:numel(x)
        r_0 = [x(i) y(i) z(i) 1].';
        r_1 = Toffset*r_0;
        x(i) = r_1(1);
        y(i) = r_1(2);
        z(i) = r_1(3);
    end

    % Create the sides
    obj = cell(6,1);
    for i = 1:numel(obj)
        obj{i} = patch(x(:,i),y(:,i),z(:,i),c(:,i),'Parent',ax,'Visible','on');
        obj{i}.EdgeColor = 'k';
        obj{i}.FaceColor = color;
        obj{i}.FaceAlpha = alpha;
    end

end
