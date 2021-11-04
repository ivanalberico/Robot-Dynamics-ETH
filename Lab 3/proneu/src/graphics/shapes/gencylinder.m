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

function [ obj ] = gencylinder(ax, radius, length, color, alpha, offsets)

    % Define data ranges
    tcyl = radius*ones(10,1);
    azimuth = linspace(0, 2*pi);
    zl = zeros(1,size(azimuth,2));
    zh = ones(1,size(azimuth,2))*length;
    cylxl = radius*cos(azimuth);
    cylyl = radius*sin(azimuth);
    cylxh = radius*cos(azimuth);
    cylyh = radius*sin(azimuth);
    [X,Y,Z] = cylinder(tcyl,50);
    Z = Z*length;
    C = zeros(size(X,1),size(X,2));
    c = zeros(size(cylxl,1),size(cylxl,2));

    % Create the position and orientation offsets
    r_off = -offsets.r;
    r_off = r_off(:);
    r_off = r_off - [0 0 length/2].';
    Toffset = eye(4);
    Toffset(1:3,4) = r_off;
    Toffset(1:3,1:3) = offsets.C;

    % Set initial desired pose of the cylinder wall
    for i=1:size(X,1)
        for j=1:size(X,2)
            r_00 = [X(i,j) Y(i,j) Z(i,j) 1].';
            r_10 = Toffset*r_00;
            X(i,j) = r_10(1);
            Y(i,j) = r_10(2);
            Z(i,j) = r_10(3);
        end
    end
    
    % Create the cylindrical shaft object
    obj = cell(3,1);
    obj{1} = cmesh(X, Y, Z, C, 'Parent', ax);
    obj{1}.EdgeColor = 'none';
    obj{1}.FaceColor = color;
    obj{1}.FaceAlpha = alpha;
    obj{1}.FaceLighting = 'gouraud';
    
    % Set initial desired pose of the cylinder bottom
    for i=1:numel(cylxl)
        r_01 = [cylxl(i) cylyl(i) zl(i) 1].';
        r_11 = Toffset*r_01;
        cylxl(i) = r_11(1);
        cylyl(i) = r_11(2);
        zl(i)   = r_11(3);
    end
    
    % Create the cylindrical bottom object
    obj{2} = patch(cylxl, cylyl, zl, c, 'Parent', ax);
    obj{2}.FaceColor = color;
    obj{2}.FaceAlpha = alpha;
    obj{2}.FaceLighting = 'gouraud';
    
    % Set initial desired pose of the cylinder top
    for i=1:numel(cylxh)
        r_02 = [cylxh(i) cylyh(i) zh(i) 1].';
        r_12 = Toffset*r_02;
        cylxh(i) = r_12(1);
        cylyh(i) = r_12(2);
        zh(i)   = r_12(3);
    end
    
    % Create the cylindrical top object
    obj{3} = patch(cylxh, cylyh, zh, c ,'Parent', ax);
    obj{3}.FaceColor = color;
    obj{3}.FaceAlpha = alpha;
    obj{3}.FaceLighting = 'gouraud';

end
