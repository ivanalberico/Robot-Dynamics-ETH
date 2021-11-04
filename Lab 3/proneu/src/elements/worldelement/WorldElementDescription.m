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

classdef WorldElementDescription
    
    % Element descriptor
    properties
        name        = 'element';
        cs          = struct('P_r_PB', sym(zeros(3,1)), 'C_PB', sym(eye(3)));
        geometry    = struct('type', 'none', ...
                             'issolid', false, ...
                             'params', [], ...
                             'values', [], ...
                             'offsets', struct('r', sym(zeros(3,1)), 'C', sym(zeros(3,3))), ...
                             'color', [0.5 0.5 0.5], ...
                             'alpha', 1.0);
    end
    
    methods
        
        % Element generator
        function [] = generateelement(obj, element)
            
            %%%
            % Check input arguments
            %%%
            
            if ~isa(obj, 'WorldElementDescription')
                error('Argument is not a WorldElementDescription type');
            end
            
            if ~isa(element, 'WorldElement')
                error('Argument is not a WorldElement type');
            end
            
            % Body geometry definitions
            element.geometry.type       = obj.geometry.type;
            element.geometry.issolid    = obj.geometry.issolid;
            element.geometry.params     = obj.geometry.params;
            element.geometry.values     = obj.geometry.values;
            element.geometry.offsets.r  = obj.geometry.offsets.r;
            element.geometry.offsets.C  = obj.geometry.offsets.C;
            element.geometry.color      = obj.geometry.color;
            element.geometry.alpha      = obj.geometry.alpha;
            
            % Body name
            element.name = obj.name;
            
        end
        
    end
    
end
