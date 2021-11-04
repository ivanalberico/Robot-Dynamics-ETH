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

classdef RobotController < handle
    
    % Controller description
    properties (Access = public)
        % Name of the controller object
        name = 'controller'; 
    end
    
    % Controller interfaces
    properties (Access = public)
        % Handle to externally defined controller function
        compute = [];
    end
    
    % Class internals
    properties (Access = private)
        % Internal handle to externally defined computation function
        controller = [];
        % Internal handle to RobotModel object
        robot = [];
        % Internal handle to WorldModel object
        world = [];
        % Internal handle to RobotModel object
        logger = [];
    end
    
    % Class operations
    methods
        
        function [ obj ] = RobotController(varargin)
            
            % Initialize arguments
            compfunc = [];
            rmodel = [];
            wmodel = [];
            dlog = [];
            
            % Parse arguments list
            if ~isempty(varargin)
                for i=1:numel(varargin)
                    if isa(varargin{i},'RobotModel')
                        rmodel = varargin{i};
                    elseif isa(varargin{i},'WorldModel')
                        wmodel = varargin{i};
                    elseif isa(varargin{i},'DataLogger')
                        dlog = varargin{i};
                    elseif isa(varargin{i},'function_handle')
                        compfunc = varargin{i};
                    end
               end
            end
            
            % Check if a RobotModel has been specified
            if ~isempty(rmodel)
                obj.robot = rmodel;
            end
            
            % Check if a WorldModel has been specified
            if ~isempty(wmodel)
                obj.world = wmodel;
            end
            
            % Check if a RobotSimulatorData has been specified
            if ~isempty(dlog)
                obj.logger = dlog;
            end
            
            % Set external controller function
            if ~isempty(compfunc)
                obj.controller = compfunc;
                obj.compute = @(t,x) obj.controller(obj.robot,obj.world,obj.logger,t,x);
            end
           
        end
        
        function [] = setcontroller(obj, controller)

            % Check arguments
            if ~isa(controller,'function_handle')
                error('Invalid argument. Argument must be of type "function_handle"');
            end

            % Set the system model
            obj.controller = controller;

            % Update the computation callback;
            obj.compute = @(t,x) obj.controller(obj.robot,obj.world,obj.logger,t,x);
            
        end
        
        function [] = setrobotmodel(obj, model)

            % Check arguments
            if ~isa(model,'RobotModel')
                error('Invalid argument. Argument must be of type "RobotModel"');
            end

            % Set the system model
            obj.robot = model;

            % Update the computation callback;
            obj.compute = @(t,x) obj.controller(obj.robot,obj.world,obj.logger,t,x);
            
        end
        
        function [] = setworldmodel(obj, model)

            % Check arguments
            if ~isa(model,'WorldModel')
                error('Invalid argument. Argument must be of type "WorldModel"');
            end

            % Set the system model
            obj.world = model;

            % Update the computation callback;
            obj.compute = @(t,x) obj.controller(obj.robot,obj.world,obj.logger,t,x);
            
        end

        function [] = setdatalogger(obj, logger)
            
            % Check arguments
            if ~isa(logger,'DataLogger')
                error('Invalid argument. Argument must be of type "RobotSimulatorData"');
            end

            % Set the system model
            obj.logger = logger;
            
            % Update the computation callback;
            obj.compute = @(t,x) obj.controller(obj.robot,obj.world,obj.logger,t,x);
            
        end
        
    end
    
end