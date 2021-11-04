classdef RigidBodyGraphics < handle
    
    % Front-end visualzation handling data
    properties (Access = public)
        % Graphics transform object handle
        tf      = [];
    end
    
    % Internal graphics data
    properties (Access = private)
        % Name of body
        name    = [];
        % Body-fixed coordiante system frame
        csf     = [];
        % Center of Mass point graphics handle
        com     = [];
        % Simple graphics handle for geometric primitives
        simple  = [];
        % Custom graphics handle for imported meshes
        custom  = [];
    end
    
    % Internal buffer containers
    properties (Access = private)
        % Name of body
        gfig = [];
        % Body-fixed coordiante system frame
        gax = [];
    end
    
    methods
        
        function [ obj ] = RigidBodyGraphics(varargin)
            % Default values
            bname = 'body';
            baxes = [];
            % Flatten nested cell arrays
            varargin = flatcells(varargin);
            % Check input arguments
            if ~isempty(varargin)
                for i=1:numel(varargin)
                    if isa(varargin{i}, 'char');
                        bname = varargin{i}; 
                    elseif isa(varargin{i}, 'matlab.graphics.axis.Axes');
                        baxes = varargin{i};
                    end
                end
            end
            % Set the body name
            obj.name = bname;
            % Set the body graphics axes
            if ~isempty(baxes)
                obj.gax = baxes;
            else
                % Create the graphics buffers if axes have not been specified
%                 obj.gfig = figure('NumberTitle', 'off','visible', 'off');
                obj.gfig = figure('NumberTitle', 'off');
                obj.gax = axes('parent', obj.gfig);
            end
        end
        
        function [] = setname(obj, name)
            % Set the body name
            if ~isempty(name)
                obj.name = name;
            end
        end
        
        function [named] = hasname(obj)
            % Check if the body has been named
            if isempty(obj.name) || strcmp(obj.name, 'none')
                named = false;
            else
                named = true;
            end
        end
        
        function [] = setaxes(obj, ax)
            % Generate new tf object to handle the objects in the specified
            % axes
            obj.tf = hgtransform('visible', 'off', 'parent', ax);
            % Set the new tf as the parent of all existing objects
            if ~isempty(obj.csf)
                for i=1:numel(obj.csf)
                   set(obj.csf{i}, 'parent', obj.tf);
                   set(obj.csf{i}, 'visible', 'on');
                end
            end
            if ~isempty(obj.com)
                for i=1:numel(obj.com)
                   set(obj.com{i}, 'parent', obj.tf);
                   set(obj.com{i}, 'visible', 'on');
                end
            end
            if ~isempty(obj.simple)
                for i=1:numel(obj.simple)
                   set(obj.simple{i}, 'parent', obj.tf);
                   set(obj.simple{i}, 'visible', 'on');
                end
            end
            if ~isempty(obj.custom)
                for i=1:numel(obj.custom)
                   set(obj.custom{i}, 'parent', obj.tf);
                   set(obj.custom{i}, 'visible', 'on');
                end
            end
        end
        
        function [] = enable(obj)
            % Enable the body graphics in the scene
            set(obj.tf, 'visible', 'on');
        end
        
        function [] = disable(obj)
            % Disable the body graphics in the scene
            set(obj.tf, 'visible', 'off');
        end
        
        function [] = remove(obj)
            % Remove all graphics objects from current axes/figure
            set(obj.tf, 'parent', obj.gax, 'visible', 'off');
            set(obj.csf, 'parent', obj.gax, 'visible', 'off');
            set(obj.com, 'parent', obj.gax, 'visible', 'off');
            set(obj.simple, 'parent', obj.gax, 'visible', 'off');
            set(obj.custom, 'parent', obj.gax, 'visible', 'off');
        end
        
        function [] = coordinateframe(obj, scale, fontsize)
            % Create the coordinate system frame object
            obj.csf = gencsframe(obj.gax,obj.name,scale,fontsize);
        end
        
        function [] = centerofmass(obj, varargin)
            %
            % TODO
            %
        end
        
        function [] = simplegraphics(obj, geometry)
            % Check for valid body geometry decription
            if ~isempty(geometry)
                if ~isa(geometry, 'RigidBodyGeometry');
                     error('Input argument is not of RigidBodyGeometry type');
                end
            end
            % Determine geometry to generate
            switch geometry.type
                case 'ellipsoid'
                    a = geometry.values(1);
                    b = geometry.values(2);
                    c = geometry.values(3);
                    offsets = geometry.offsets;
                    rgb = geometry.color;
                    alpha = geometry.alpha;
                    obj.simple = genellipsoid(obj.gax, a, b, c, rgb, alpha, offsets);
                case 'sphere'
                    r = geometry.values(1);
                    offsets = geometry.offsets;
                    rgb = geometry.color;
                    alpha = geometry.alpha;
                    obj.simple = gensphere(obj.gax, r, rgb, alpha, offsets);
                case 'cylinder'
                    r = geometry.values(1);
                    l = geometry.values(2);
                    offsets = geometry.offsets;
                    rgb = geometry.color;
                    alpha = geometry.alpha;
                    obj.simple = gencylinder(obj.gax, r, l, rgb, alpha, offsets);
                case 'rod'
                    l = geometry.values(1);
                    t = geometry.values(2);
                    offsets = geometry.offsets;
                    rgb = geometry.color;
                    obj.simple = genrod(obj.gax, l, t, rgb, offsets);
                case 'cuboid'
                    l = geometry.values(1);
                    w = geometry.values(2);
                    h = geometry.values(3);
                    offsets = geometry.offsets;
                    rgb = geometry.color;
                    alpha = geometry.alpha;
                    obj.simple = gencuboid(obj.gax, l, w, h, rgb, alpha, offsets);
                case 'particle'
                    rgb = geometry.color;
                    alpha = 1.0;
                    obj.simple = genparticle(obj.gax, rgb, alpha);
                case 'plane'
                    w = geometry.values(1);
                    l = geometry.values(2);
                    offsets = geometry.offsets;
                    rgb = geometry.color;
                    alpha = geometry.alpha;
                    obj.simple = genplane(obj.gax, l, w, rgb, alpha, offsets);
                case 'none'
                    % Do nothing in this case.
                otherwise
                    error('Invalid geometry.type value.');
            end
        end
        
        function [] = customgraphics(obj, filename, format, scale, offsets, color)
            % Determine the format of the offsets
            if isstruct(offsets) && isfield(offsets, 'r') && isfield(offsets, 'C')
                r = offsets.r;
                C = offsets.C;
            elseif isnumeric(offsets) && (size(offsets,1) == 4) && (size(offsets,2) == 4)
                r = offsets(1:3,4);
                C = offsets(1:3,1:3);
            else
                error('Invalid offset value. Must be struct with *.r and *.C or 4x4 homogeneous transform.');
            end
            % Check file format and use appropriate geometry import tool
            switch format
                case 'stl'
                    obj.custom = stlmesh(obj.gax, filename, scale, r, C, color, alpha);
                case 'obj'
                    obj.custom = objmesh(obj.gax, filename, scale, r, C, color, alpha);
                otherwise
                    error('Invalid format value.');
            end
        end
        
    end
    
end
