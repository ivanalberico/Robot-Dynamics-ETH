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

function [ cellsout ] = flatcells(cellsin)
% FLATCELLS is a helper function to flatten nested cell arrays. 
% 
% FLATCELLS(cellsin) searches every cell element in cellist and put them on
% the top most level. Therefore, FLATCELLS linearizes a cell array tree
% structure. 

    if iscell(cellsin)
        cellsout = flattencellarray(cellsin);
    else
        cellsout = cellsin;
    end

end

function [ output ] = flattencellarray(input)
    
    output = {};

    for k = 1:numel(input)
        if iscell(input{k})
            output = [output flattencellarray(input{k})];
        else
            output = [output input(k)];
        end
    end

end



