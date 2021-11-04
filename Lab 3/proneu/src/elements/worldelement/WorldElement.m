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

classdef WorldElement < handle
    
    % Class interface
    properties 
        % Name of the world element
        name = 'element';
        %
        geometry = WorldElementGeometry;
        %
        graphics = [];
    end
    
    % Class operations
    methods
        
        function [ obj ] = WorldElement(varargin)
            
            % Initialize body descriptor
            elementdes = [];
            
            % Check varargin for valid body decription
            if ~isempty(varargin)
                
                % Remove nested cell arguments
                varargin = flatcells(varargin);
                
                % Check for a valid descriptor
                for i=1:numel(varargin)
                    if isa(varargin{i}, 'WorldElementDescription');
                         elementdes = varargin{i};
                    end
                end
            end
            
            % Throw error if a valid body description has not been defined
            if isempty(elementdes)
                error('Input argument is not of any type "WorldElementDescription".');
            end
            
            % Generate body if valid descriptor has been specified
            elementdes.generateelement(obj);
            
        end
        
        % Helper function to create graphics objects for the element visualization
        function [] = creategraphics(obj, varargin)
            
            % Remove nested cell arguments
            varargin = flatcells(varargin);
            
            % Create the element graphics object
            obj.graphics = WorldElementGraphics(obj.name, varargin);
        end 
        
    end
    
end