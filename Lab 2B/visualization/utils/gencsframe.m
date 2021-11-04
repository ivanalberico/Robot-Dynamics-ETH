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

function [ obj ] = gencsframe(ax, name, scale, fontsize)

    % Create arrow data
    csf_x = 0;
    csf_y = 0;
    csf_z = 0;
    csf_u = scale*[0 1 0 0];
    csf_v = scale*[0 0 1 0];
    csf_w = scale*[0 0 0 1];

    % Create labels data
    csf_olabel = strcat('$O_{',name,'}$');
    csf_xlabel = strcat('${\bf e}^{',name,'}_{x}$');
    csf_ylabel = strcat('${\bf e}^{',name,'}_{y}$');
    csf_zlabel = strcat('${\bf e}^{',name,'}_{z}$');
    csf_labels = {{csf_olabel},{csf_xlabel},{csf_ylabel},{csf_zlabel}};

    % Create latex label positions
    randmax = scale/20;
    randoff = (randmax)*rand(4);
    csf_xlabel_pos = [csf_x;csf_x;csf_x;csf_x]+csf_u.'+randoff(1);
    csf_ylabel_pos = [csf_y;csf_y;csf_y;csf_y]+csf_v.'+randoff(2);
    csf_zlabel_pos = [csf_z;csf_z;csf_z;csf_z]+csf_w.'+randoff(3);

    % Allocate the object cells
    obj = cell(7,1);

    % Generate the arrow objects
    hold(ax,'on');
    obj{1} = quiver3(csf_x,csf_y,csf_z,csf_u(2),csf_v(2),csf_w(2),0,'Parent',ax);
    obj{2} = quiver3(csf_x,csf_y,csf_z,csf_u(3),csf_v(3),csf_w(3),0,'Parent',ax);
    obj{3} = quiver3(csf_x,csf_y,csf_z,csf_u(4),csf_v(4),csf_w(4),0,'Parent',ax);
    hold(ax,'off');

    % Configure quiver frame format properties
    obj{1}.Color = 'red';
    obj{2}.Color = 'green';
    obj{3}.Color = 'blue';
    obj{1}.LineWidth = 1.5;
    obj{2}.LineWidth = 1.5;
    obj{3}.LineWidth = 1.5;
    obj{1}.MaxHeadSize = 0.2;
    obj{2}.MaxHeadSize = 0.2;
    obj{3}.MaxHeadSize = 0.2;

    % Generate the label objects
    h = text(csf_xlabel_pos,csf_ylabel_pos,csf_zlabel_pos,csf_labels, ...
             'Interpreter','latex','FontSize',fontsize,'Parent',ax);

         % Store the generated graphics handles
    obj{4} = h(1);
    obj{5} = h(2);
    obj{6} = h(3);
    obj{7} = h(4);

end
