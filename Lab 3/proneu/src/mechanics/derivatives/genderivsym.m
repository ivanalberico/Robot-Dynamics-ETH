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

function dx = genderivsym( x )

    % Get dimensions of x and initialize the derivative
    Nx = size(x);
    dx = sym(zeros(Nx(1),Nx(2)));

    % Generate the derivative elements
    for i=1:Nx(1)
        for j=1:Nx(2)
            xstr = char(x(i,j));
            dxstr = strcat('d',xstr);
            dx(i,j) = sym(dxstr);
        end
    end
end
