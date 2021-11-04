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

classdef RobotVisualization < handle
    
    % Class data
    properties (Access = public)
        %
        name = 'RobotVis';
        %
        rvfigure = [];
        %
        rvaxes = [];
    end
    
    % Class internals
    properties (Access = private)
        % An internal handle to the target robot
        robot = [];
        % An internal handle to the target world
        world = [];
        % Number of WorldElement objects
        Nwe = [];
        % Number of RigidBody objects
        Nrb = [];
        % Number of ForceTorque objects
        Nft = [];
        % Number of RobotModel configuration variables
        Nq = [];
        % Current RobotModel parameters
        robotparams = [];
        % Current RobotModel configuration
        robotconfiguration = [];
        % Visualization configuration
        fontsize = 10;
        csfscale = 0.2;
        zoom = 1.0
        posoffset = [0 0 0].'
        % Optional callback function
        cfunc = [];
        callback = [];
    end
    
    % Class operations
    methods
        
        function [ obj ] = RobotVisualization(robot, world, fontsize, csfscale, zoom, posoffset)
            
            % Iniailize the internal robot model handles
            if isa(robot, 'RobotModel')
                obj.robot = robot;
            else
                error('Invalid argument. Input must be of type "RobotModel".');
            end
            
            % Set the world model handle if specified
            if isa(world, 'WorldModel')
                obj.world = world;
            end
            
            % Set the number of graphics objects
            if ~isempty(obj.robot.body)
                obj.Nrb = numel(obj.robot.body);
            else
                obj.Nrb = 0;
            end
            
            if ~isempty(obj.robot.force)
                obj.Nft = numel(obj.robot.force);
            else
                obj.Nft = 0;
            end
            
            if ~isempty(obj.robot.dynamics.symbols)
                obj.Nq = numel(obj.robot.dynamics.symbols.q);
            else
                obj.Nq = 0;
            end
            
            % Iniailize the internal world model handles
            if ~isempty(obj.world)
                obj.Nwe = length(obj.world.element);
            else
                obj.Nwe = 0;
            end
            
             % Set the visualization configuration
            if ~isempty(fontsize)
                obj.fontsize = fontsize;
            end
            if ~isempty(csfscale)
                obj.csfscale = csfscale;
            end
            if ~isempty(zoom)
                obj.zoom = 1/zoom;
            end
            if ~isempty(posoffset)
                obj.posoffset = posoffset;
            end
            
        end
        
        function [] = open(obj)
            
            % Set the figure name
            figurename = strcat(obj.name,':: ',obj.robot.name);
            
            % Generate new figure and axes objects
            obj.rvfigure = figure('Name', figurename, 'NumberTitle', 'off');
            obj.rvaxes = axes('parent', obj.rvfigure);
            
            % Define the field of view to be a box
            viewx = [-2 2] + [obj.posoffset(1) obj.posoffset(1)];
            viewy = [-2 2] + [obj.posoffset(2) obj.posoffset(2)];
            viewz = [-2 2] + [obj.posoffset(3) obj.posoffset(3)];
            viewlim3d = [viewx viewy viewz];
            
            % Set the default configuration
            axis(obj.rvaxes, 'equal');
            axis(obj.rvaxes, 'vis3d');
            axis(obj.rvaxes, 'off');
            axis(obj.rvaxes, 'auto');
            axis(obj.rvaxes, obj.zoom*viewlim3d);
            camlight('left');
            
        end
        
        function [] = close(obj)
            %
            % TODO
            %
        end
        
        function [] = configure(obj,varargin)
            %
            % TODO
            %
        end
        
        function [] = setcallback(obj,func)
            obj.cfunc = func;
            obj.callback = @(t,x,v) obj.cfunc(obj,obj.robot,obj.world,t,x,flatcells(v));
        end
        
        function [] = load(obj)
            
            % Get the numerical values of the model parameters
            if ~isempty(obj.robot.parameters.values)
                obj.robotparams = obj.robot.parameters.values;
            else
               error('Model parameters are unspecified. Please define them in "robotmdl.parameters.values".'); 
            end
            
            % Set the initial state and store internally
            qinit = zeros(obj.Nq, 1);
            
            % Correct for the type of base orientation parametrization
            if strcmp(obj.robot.description.orientation, 'quaternion')
                qinit(4) = 1;
            elseif strcmp(obj.robot.description.orientation, 'angleaxis')
                qinit(7) = eps;
            end
            
            % Store the state internally
            obj.robotconfiguration = qinit;
            
            % Generate the RigidBody graphics
            for i=1:obj.Nrb
                obj.robot.body(i).creategraphics(obj.rvaxes);
                % Enable local CSF if a name as been defined;
                if (obj.robot.body(i).graphics.hasname() == true)
                    obj.robot.body(i).graphics.coordinateframe(obj.csfscale, obj.fontsize);
                end
                obj.robot.body(i).graphics.simplegraphics(obj.robot.body(i).geometry);
                obj.robot.body(i).graphics.setaxes(obj.rvaxes);
                obj.robot.body(i).graphics.enable();
                Tviz = obj.robot.body(i).kinematics.compute.T_IBi(qinit,[],[],obj.robotparams);
                obj.robot.body(i).graphics.tf.Matrix = Tviz;
            end
            
            % Generate the ForceTorque graphics
            for j=1:obj.Nft
                %
                % TODO
                %
            end
            
            % Generate the WorldElement graphics
            for k=1:obj.Nwe
                obj.world.element(k).creategraphics(obj.rvaxes);
                obj.world.element(k).graphics.simplegraphics(obj.world.element(k).geometry);
                obj.world.element(k).graphics.setaxes(obj.rvaxes);
                obj.world.element(k).graphics.enable();
                %
                % TODO
                %
            end
            
        end
        
        function [] = update(obj, qdata, varargin)
            
            % Update current configuration
            obj.robotconfiguration = qdata;
            
            % Update RigidBody graphics
            for i=1:obj.Nrb
                Tviz = obj.robot.body(i).kinematics.compute.T_IBi(qdata,[],[],obj.robotparams);
                obj.robot.body(i).graphics.tf.Matrix = Tviz;
            end

            % Update ForceTorque graphics
            for j=1:obj.Nft
                %
                % TODO
                %
            end

            % Update WorldElement graphics
            for k=1:obj.Nwe
                %
                % TODO
                %
            end

            % Invoke optional callback
            if ~isempty(obj.callback)
               obj.callback(0.0, qdata, varargin);
            end
            
            % Update figure/axes
            drawnow;
            
        end
        
        function [] = clear(obj)
            %
            % TODO
            %
        end
        
        function [] = reset(obj)
            %
            % TODO
            %
        end
        
        function [] = save(obj)
            %
            % TODO
            %
        end
        
        function [] = export(obj)
            %
            % TODO
            %
        end
        
    end
    
end

%
% HELPER FUNCTIONS
%
