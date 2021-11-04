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

classdef OdeSystem < handle
    
    % Model interface
    properties (Access = public)
        % The symbolic properties of the system
        symbols = [];
        % Numerical function interfaces for computation 
        compute = [];
    end
    
    % Model parameters
    properties (Access = public)
        % Numerical parameters
        parameters = struct('symbols', [], 'values', []);
        % Initial conditions
        xinit = []; 
    end
    
    % Model internals
    properties (Access = private)
        % Internal handle to a RobotModel object
        robot = [];
    end
     
    % Model operations
    methods
        
        function [ obj ] = OdeSystem(model, varargin)
            
            proneu_info('Creating OdeSystem object.');
            
            proneu_info('Checking arguments.');
            
            % Check arguments
            check_arguments(obj, model, varargin);
            
            proneu_info('Generating ODE properties.');
            
            % Instantiate the symbols member
            obj.symbols = OdeSystemProperties;
            
        end
        
        function [] = generatemodel(obj)
            
            proneu_info('Generating OdeSystem model representation.');
            
            % Check the type of multi-rigid-body
            if ~strcmp(obj.robot.description.type, 'fixed')
                error('This class only supports Fixed-Base systems.');
            else
                generate_system(obj);
            end
            
        end
        
        function [] = simplifydynamics(obj, cmode, ns)
            
            % Check that we all elements are symbols
            if ~isa(obj.symbols.M, 'sym') || ~isa(obj.symbols.f, 'sym')
                error('Simplifications cannot be performed on non-symbolic elements.');
            end
            
            %%% Simplify basic dynamics properties
            
            proneu_info('Performing symbolic simplifications on the system dynamics: ');
            
            proneu_info('-->Simplifying M ');
            tstart = tic;
            obj.M = simplesym(obj.symbols.M, cmode, ns);
            tsimp = toc(tstart);
            proneu_info(['---->Duration: ' num2str(tsimp) ' s']);
            
            proneu_info('-->Simplifying f ');
            tstart = tic;
            obj.b = simplesym(obj.symbols.f, cmode, ns);
            tsimp = toc(tstart);
            proneu_info(['---->Duration: ' num2str(tsimp) ' s']);
            
        end
        
        function [] = generatefunctions(obj, varargin)
            
            proneu_info('Generating numeric functions for the ODE System. ');
            
            % Initialize settings
            values = [];
            
            % Check arguments
            if ~isempty(varargin)
                for i=1:numel(varargin)
                    if ischar(varargin{i}) && strcmp(varargin{i}, 'values')
                        values = varargin{i+1};
                    end    
                end
            end
            
            % Set the system parameters array
            params = obj.robot.parameters.symbols(:);
            
            % Generate a symbolic time variable
            syms t real;
            assume(t>0);
            
            % Get the system definition
            x = obj.robot.dynamics.symbols.q(:);
            dx = obj.robot.dynamics.symbols.dq(:);
            u_a = obj.robot.dynamics.symbols.tau_a(:); 
            u_e = obj.robot.dynamics.symbols.tau_e(:);
            
            % Store a local copy of the model symbols
            syssymbols = obj.symbols;
            
            % If "values" have been specified then generate non-parametrized functions
            if ~isempty(values)
                syssymbols.M = subs(syssymbols.M, params, values);
                syssymbols.f = subs(syssymbols.f, params, values);
                mvars = {t, [x; dx]};
                fvars = {t, [x; dx], u_a, u_e};
            else
                mvars = {t, [x; dx], params};
                fvars = {t, [x; dx], u_a, u_e, params};
            end
            
            % Generate matlab functions for each kinematic quantity
            obj.compute.M = matlabFunction(syssymbols.M, 'vars', mvars);
            obj.compute.f = matlabFunction(syssymbols.f, 'vars', fvars);
            
        end
        
    end
    
end

%
% HELPER FUNCTIONS
%

function [] = check_arguments(obj, model, varargin)

    % Check processed arguments
    if ~isempty(model)
        if isa(model, 'RobotModel')
            obj.robot = model;
        else
            error('Invalid or missing robot model specified. Argument should be of type "RobotModel".');
        end
    end

end

function [] = generate_system(obj)
    
    % Set system definitions
    q = obj.robot.dynamics.symbols.q(:);
    dq = obj.robot.dynamics.symbols.dq(:);
    ddq = obj.robot.dynamics.symbols.ddq(:);

    % Define the system state vectors
    obj.symbols.q  = q(:);
    obj.symbols.x  = [q(:); dq(:)];
    obj.symbols.dx = [dq(:); ddq(:)];

    % Define the system input vectors
    obj.symbols.u_int = obj.robot.dynamics.symbols.tau_a;
    obj.symbols.u_ext = obj.robot.dynamics.symbols.tau_e;

    % Store local copy of symbolic RigidBody dynamics quantities
    M_mbs = obj.robot.dynamics.symbols.M;
    f_mbs = obj.robot.dynamics.symbols.tau - obj.robot.dynamics.symbols.b - obj.robot.dynamics.symbols.g;

    % Formulate ODE 
    M_ds = [eye(size(M_mbs)) zeros(size(M_mbs)); zeros(size(M_mbs)) M_mbs];
    f_ds = [dq(:); f_mbs(:)];

    % Store all ODE/DAE symbols internally
    obj.symbols.M = M_ds;
    obj.symbols.f = f_ds;

end
