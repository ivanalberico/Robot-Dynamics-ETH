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

classdef RobotSimulator < handle
    
    % Simulator description
    properties (Access = private)
        % description struct
        description = struct('solver', []);
    end
    
    % Simulator subsystems
    properties (Access = public)
        % Numerical solver/integrator
        solver = [];
        % System interface
        system = [];
        % Simulation data logger
        data = [];
        % Simulation data scope viewer
        scope = [];
    end
    
    % Simulator internal model handles
    properties (Access = private)
        % Handle to RobotModel object
        robot = [];
        % Handle to RobotController object
        controller = [];
        % Handle to WorldModel object
        world = [];
        % Handle to RobotVisualization object
        visualization = [];
    end
    
    % Simulator internal data
    properties (Access = private)
        % Number of system configurations variables
        Nq = [];
        % Number of system state variables
        Nx = [];
        % Number of internal system input variables
        Nui = [];
        % Number of external system input variables
        Nue = [];
        % Internal buffer for simulation data
        simbuffer = struct('time', [], 'data', []);
        % 3D Visualization enable flag;
        visenabled =  false;
        % Scope enable flag
        scenabled = false;
    end
    
    % Class operations
    methods
        
        function [ obj ] = RobotSimulator(model, varargin)
            
            proneu_info('Setting up simulation instance.');
            
            proneu_info('-->Checking input arguments.');
            
            % Check arguments
            check_arguments(obj, model, varargin);
            
            proneu_info('-->Generating simulation environment.');
            
            %%%
            % Generate & setup numerical solver
            %%%
            
            % Use solver description for selection
            switch (obj.description.solver)
                case 'kinematics'
                    % Generate fixed-step ODE solver
                    obj.solver = KinematicsSolver;
                    obj.system = OdeSystem(model);
                case 'fixedstep'
                    % Generate fixed-step ODE solver
                    obj.solver = FixedStepSolver;
                    obj.system = OdeSystem(model);
                case 'dormandprince'
                    % Generate variable-step ODE solver
                    obj.solver = DormandPrinceSolver;
                    obj.system = OdeSystem(model);
                case 'fsfb'    
                    obj.solver = FixedStepFloatingBaseSolver;
                    obj.system = FloatingBaseSystem(model);
                case 'moreau'
                    obj.solver = MoreauTimeSteppingSolver; 
                    obj.system = FloatingBaseSystem(model);
                otherwise
                    error('Invalid "solver" type specified.');
            end
            
            proneu_info(['Simulation will use a ' obj.description.solver ' solver type.']);
            
            %%%
            % Generate & configure system model
            %%%
            
            proneu_info('Generating system model.');
            
            % Generate the numerical model of the system
            obj.system.generatemodel();
            obj.system.generatefunctions();
            
            % Initialize the system state and input vector sizes;
            obj.Nq = length(obj.system.symbols.q);
            obj.Nx = length(obj.system.symbols.x);
            obj.Nui = length(obj.system.symbols.u_int);
            obj.Nue = length(obj.system.symbols.u_ext);
            
            %%%
            % Generate & setup simulation data logging
            %%%
            
            proneu_info('Generating data logger.');
            
            % Generate simulation data logger
            obj.data = DataLogger();
            
            proneu_info('Generating data scope.');
            
            % Generate a simulation data plotter
            obj.scope = DataScope(model);
            
            %%%
            % Sub-system connections
            %%%
            
            % Connect the robot controller with the robot model and logger
            if ~isempty(obj.controller)
                obj.controller.setrobotmodel(obj.robot);
                obj.controller.setdatalogger(obj.data);
            else
                % Set all internal inputs to zero if no controller has been
                % specified
                obj.controller = RobotController;
                obj.controller.compute = @(t,x) zeros(obj.Nui,1);
            end
            
            % Connect the world model with the robot model and logger
            if ~isempty(obj.world)
                obj.world.setrobotmodel(obj.robot);
                obj.world.setdatalogger(obj.data);
            else
                % Set all external inputs to zero if no controller has been
                % specified
                obj.world = WorldModel;
                obj.world.compute = @(t,x) zeros(obj.Nue,1);
            end
            
        end
        
        function [] = setup(obj, varargin)
            
            proneu_info('Setting up simulation instance.');
            
            % Initialize simulation parameters
            regenerate = false;
            xinit = [];
            parameters = [];
            
            % Parse arguments list
            if ~isempty(varargin)
                for i=1:numel(varargin)
                    if ischar(varargin{i}) && strcmp(varargin{i}, 'xinit')
                        xinit = varargin{i+1};
                    elseif ischar(varargin{i}) && strcmp(varargin{i}, 'parameters')
                        parameters = varargin{i+1};
                    elseif ischar(varargin{i}) && strcmp(varargin{i}, 'regenerate')
                        regenerate = true;
                    end
               end
            end
            
            % Check for user-specified parameters
            if (regenerate == true)
                obj.system.generatemodel();
                obj.system.generatefunctions();
                obj.Nx = length(obj.system.symbols.x);
                obj.Nui = length(obj.system.symbols.u_int);
                obj.Nue = length(obj.system.symbols.u_ext);
            end
            if isempty(xinit)
                warning('Initial conditions are unspecified. Will use default zero values');
                xinit = zeros(obj.Nx,1);
            end
            if ~isempty(parameters)
                obj.robot.parameters.values = parameters;
                obj.system.parameters.values = parameters;
            else
                obj.system.parameters.values = obj.robot.parameters.values;
            end
            
            % Set the simulation initial conditions
            obj.system.xinit = xinit;
            
            % Configure the simulation run-time operations
            uifunc = obj.controller.compute;
            uefunc = obj.world.compute;
            sparams = obj.system.parameters.values;
            obj.solver.setup(obj.Nx, obj.Nq, obj.system.compute, uifunc, uefunc, sparams);
            
        end
        
        function [] = run(obj, conf)
            
            proneu_info('Preparing simulation run.');
            
            % Set the visualization flags
            scope_enabled = obj.scenabled;
            visual_enabled = obj.visenabled;
            
            % Initialize simulation parameters
            tstep   = conf.tstep;
            tstart  = conf.tstart;
            tstop   = conf.tstop;
            tvis    = 0.0;
            fps     = conf.fps;
            init    = obj.system.xinit;
            
            % Initialize simulation time
            simtime = linspace(tstart, tstop, uint64((tstop - tstart)/tstep));
            Nt = length(simtime);
            
            % Initialize data buffer
            Nstates = obj.Nx;
            Ninputs = obj.Nui + obj.Nue;
            Ndata = Nstates + Ninputs;
            simdata = zeros(Ndata,Nt);
            comptime = zeros(Nt,1);
            simdata(1:Nstates,1) = init(:);
            
            % Set initial conditions
            currentstate = init(:);
            ts = tstart;
            tf = tstart + tstep;
            
            % Update visualization
            if (visual_enabled == true)
                qdata = currentstate(1:obj.Nq);
                obj.visualization.update(qdata);
                tvis = 0.0;
            end
            
            proneu_info('Executing simulation...');
            
            % Execute simulation loop
            for k=2:Nt
                try
                    % Start computation time counter
                    tcompbegin = tic;
                    
                    % Compute forward dynamics
                    [sdata, stime, ctime] = obj.solver.compute(currentstate, ts, tf, tstep);
                    
                    % Update integrator time window
                    ts = tf;
                    tf = tf + tstep;
                    
                    % Store latest simulation data output
                    currentstate = sdata(1:Nstates,end);
                    %simdata(:,k) = sdata(:,end);
                    simdata(1:Nstates,k) = sdata(:,end);
                    comptime(k) = ctime;
                    
                    % Post computation time keeping
                    tcompduration = toc(tcompbegin);
                    if tcompduration < tstep
                        pause(tstep-tcompduration);
                        tvis = tvis + tstep;
                    else
                        tvis = tvis + tcompduration;
                    end
                    
                    % Update scope
                    if (scope_enabled == true) && (tvis >= (1/fps))
                        obj.scope.update(sdata(:,end));                        
                    end    
                        
                    % Update visualization
                    if (visual_enabled == true) && (tvis >= (1/fps))
                        qdata = currentstate(1:obj.Nq);
                        obj.visualization.update(qdata);
                        tvis = 0.0;
                    end
                    
                catch ME
                    if strcmp(ME.identifier,'MATLAB:handle_graphics:exceptions:UserBreak')
                        disp('Operation termianted by user.');
                        break;
                    else
                        warning('Operation terminated from simulation error event:');
                        warning(ME.message);
                        break;
                    end
                end
                
            end
            
            proneu_info('Simulation ended.');
            proneu_info(['Average computaion time is: ' num2str(mean(comptime))]);
            
            % Store collected simulation data into internal buffer
            obj.simbuffer.data = simdata;
            obj.simbuffer.time = simtime;
            
        end
        
        function [] = savedata(obj)
            
            proneu_info('Storing RobotSimulator data.');
            
            %
            % TODO
            %
        end
        
        function [] = setvisualizer(obj, visualization)
           
            proneu_info('Setting RobotSimulator visualization.');
            
            if ~isempty(visualization)
                if isa(visualization, 'RobotVisualization') 
                    obj.visualization = visualization;
                    obj.visenabled = true;
                else
                    error('Invalid argument. Inut must be of type "RobotVisualization".'); 
                end
            else
                error('Empty argument. Inut must be of type "RobotVisualization".'); 
            end
            
        end
        
    end
    
end

%
% HELPER FUNCTIONS
%

function [] = check_arguments(obj, model, varargin)
    
    % Restore arguments
    varargin = flatcells(varargin);
    
    %%% Process input arguments

    % Default setup
    controller = [];
    world = [];
    solver = [];
    
    % Parse arguments
    if ~isempty(varargin)
        for i=1:numel(varargin)
            if isa(varargin{i}, 'RobotController')
                controller = varargin{i};
            elseif isa(varargin{i}, 'WorldModel')
                world = varargin{i};
            elseif ischar(varargin{i}) && strcmp(varargin{i}, 'solver')
                solver = varargin{i+1};
            end    
        end
    end
    
    %%% Check model objects

    if ~isempty(model)
        obj.robot = model;
    else
        error('A RobotModel has not been specified.');
    end

    if ~isempty(controller)
        obj.controller = controller;
    else
        warning('A RobotController has not been specified. All joint actions will be disabled.');
    end

    if ~isempty(world)
        obj.world = world;
    else
        warning('A WorldModel has not been specified. All external influences will be disabled.');
    end
    
    if ~isempty(solver)
        obj.description.solver = solver;
    else
        warning('A solver has not been specified. Defaulting to "DormandPrinceSolver"');
        obj.description.solver = 'dormandprince';
    end

end
