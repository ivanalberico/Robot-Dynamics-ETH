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

function [ obj ] = genarrowvec(ax, name, fontsize, width, color, origin, vector)

    % Create labels data
    vec_label = strcat('$\vec{\bf{r}}_{',name,'}$');

    % Create latex label positions
    randoff = 0.0005*rand(4);
    vec_label_pos = 0.75*vector + origin + randoff(3);

    % Allocate the object cells
    obj = cell(2,1);

    % Generate the arrow object
    hold(ax,'on');
    obj{1} = quiver3(origin(1),origin(2),origin(3),vector(1),vector(2),vector(3),0,'Parent',ax);
    hold(ax,'off');

    % Configure quiver frame format properties
    obj{1}.Color = color;
    obj{1}.LineWidth = width;
    obj{1}.MaxHeadSize = 0.2;

    % Generate the label objects
    h = text(vec_label_pos(1),vec_label_pos(2),vec_label_pos(3),vec_label,'Interpreter','latex','FontSize',fontsize,'Parent',ax);

    % Store the generated graphics handles
    obj{2} = h;

end
