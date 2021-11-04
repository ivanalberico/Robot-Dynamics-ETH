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

function [ obj ] = genparticle(ax, color, alpha)

    % Create the cylindrical shaft
    [X,Y,Z] = sphere(5);
    C = zeros(size(X,1),size(X,2));
    radius = 2e-2;

    % Creat the sphere object
    obj = cell(1,1);
    obj{1} = cmesh(radius*X, radius*Y, radius*Z, C, 'Parent', ax);
    obj{1}.EdgeColor = 'k';
    obj{1}.FaceColor = color;
    obj{1}.FaceAlpha = alpha;
    obj{1}.FaceLighting = 'gouraud';
end
