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

function [ obj ] = objmesh(ax, filename, scale, r, C, color, alpha)

    % Generate data from the specified file
    objdata = read_wobj(filename);

    % Create temporary figures and axes to contain the temporary patch
    tempfig = figure('visible', 'off');
    tempax = axes('Parent', tempfig);
    patchinput.vertices = objdata.vertices;
    patchinput.faces = objdata.objects(4).data.vertices;
    patchoutput = patch(patchinput,'visible','off', 'Parent', tempax);

    % Extract XYZ data from patch faces and vertices
    x = patchoutput.XData;
    y = patchoutput.YData;
    z = patchoutput.ZData;
    c = ones(1,length(x));

    % Clear unnecessary data
    clear tempax;
    clear tempfig;
    clear patchoutput;

    % Correct scaling
    x = x*scale;
    y = y*scale;
    z = z*scale;

    % Correct scaling
    x = x - r(1);
    y = y - r(2);
    z = z - r(3);

    % Correct rotation offsets
    Toff = eye(4);
    Toff(1:3,4) = zeros(3,1);
    Toff(1:3,1:3) = C;
    for i=1:size(x,1)
        for j=1:size(x,2)
            r0 = [x(i,j) y(i,j) z(i,j) 1].';
            r1 = Toff*r0;
            x(i,j) = r1(1);
            y(i,j) = r1(2);
            z(i,j) = r1(3);
        end
    end

    % Create and store patch data
    obj = cell(1,1);
    obj{1} = patch(x, y, z, c, 'visible','off', 'Parent', ax);
    obj{1}.EdgeColor = 'none';
    obj{1}.FaceColor = color;
    obj{1}.FaceAlpha = alpha;
end
