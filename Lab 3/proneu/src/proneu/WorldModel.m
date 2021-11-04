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

classdef WorldModel < handle
    
    % Model description
    properties (Access = public)
        % Name of the controller object
        name = 'world';
    end
    
    % Model elements
    properties (Access = public)
        % Array of RigidBody elements
        element = WorldElement.empty;
    end
    
    % Model interfaces
    properties (Access = public)
        % Controller computation callback
        compute = [];
    end
    
    % Class internals
    properties (Access = private)
        % Internal handle to externally defined computation function
        model = [];
        % Internal handle to RobotModel object
        robot = [];
        % Internal handle to RobotModel object
        logger = [];
    end
    
    % Class operations
    methods
        
        function [ obj ] = WorldModel(varargin)
            
            % Initial greeting
            proneulogo();
            proneu_info('Starting proNEu world creation engine...');
            
            % Check if type and method have been defined by user
            proneu_info('Processing and checking arguments...');
            
            % Initialize arguments
            elements = [];
            name = [];
            compfunc = [];
            rmodel = [];
            dlog = [];
            
            % Parse arguments list
            if ~isempty(varargin)
                for i=1:numel(varargin)
                    if isa(varargin{i},'WorldElementDescription')
                        elements = varargin{i};
                    elseif isa(varargin{i},'RobotModel')
                        rmodel = varargin{i};
                    elseif isa(varargin{i},'function_handle')
                        compfunc = varargin{i};
                    elseif isa(varargin{i},'RobotSimulatorData')
                        dlog = varargin{i};
                    elseif ischar(varargin{i}) && strcmp(varargin{i}, 'name')
                        name = varargin{i+1};
                    end
               end
            end
            
            % Set model name if specified
            if ~isempty(name)
                obj.name = name;
            end
            
            % System specific output
            proneu_info(['Generating a WorldModel for system named: "' obj.name '"']);
            
            % Set the WorldElement objects
            if ~isempty(elements)
                Nwe = length(elements);
                for k=1:Nwe
                    obj.element(k) = WorldElement(elements(k));
                end
            end
            
            % Check if a RobotModel has been specified
            if ~isempty(rmodel)
                proneu_info(['Assigning a RobotModel instance to world. Model is: "' rmodel.name '"']);
                obj.robot = rmodel;
            end
            
            % Check if a RobotSimulatorData has been specified
            if ~isempty(dlog)
                proneu_info(['Assigning a DataLogger instance to world. Logger is: "' dlog.name '"']);
                obj.logger = dlog;
            end
            
            % Set external controller function
            if ~isempty(compfunc)
                proneu_info('Assigning a computation callback to world.');
                obj.model = compfunc;
                obj.compute = @(t,x) obj.model(obj.robot,obj.logger,t,x);
            end
            
        end
        
        function [] = setcomputationmodel(obj, model)
            
            % Check arguments
            if ~isa(model,'function_handle')
                error('Invalid argument. Argument must be of type "function_handle"');
            end
            
            proneu_info('Assigning a computation callback to world.');
            
            % Set the system model
            obj.model = model;

            % Update the computation callback;
            obj.compute = @(t,x) obj.model(obj.robot,obj.logger,t,x);
            
        end
        
        function [] = setrobotmodel(obj, model)
            
            % Check arguments
            if ~isa(model,'RobotModel')
                error('Invalid argument. Argument must be of type "RobotModel"');
            end
            
            proneu_info(['Assigning a RobotModel instance to world. Model is: "' model.name '"']);
            
            % Set the system model
            obj.robot = model;

            % Update the computation callback;
            obj.compute = @(t,x) obj.model(obj.robot,obj.logger,t,x);
            
        end

        function [] = setdatalogger(obj, logger)
            
            % Check arguments
            if ~isa(logger,'DataLogger')
                error('Invalid argument. Argument must be of type "RobotSimulatorData"');
            end
            
            proneu_info('Assigning a DataLogger instance to world.');
            
            % Set the system model
            obj.logger = logger;
            
            % Update the computation callback;
            obj.compute = @(t,x) obj.model(obj.robot,obj.logger,t,x);
            
        end
        
    end
    
end

%
% HELPER FUNCTIONS
%
