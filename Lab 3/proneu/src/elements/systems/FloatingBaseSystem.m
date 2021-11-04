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

classdef FloatingBaseSystem < handle
    
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
        
        function [ obj ] = FloatingBaseSystem(model, varargin)
            
            proneu_info('Creating FloatingBaseSystem object.');
            
            proneu_info('Checking arguments.');
            
            % Check arguments
            check_arguments(obj, model, varargin);
            
            % Instantiate the symbols member
            obj.symbols = FloatingBaseSystemProperties;
            
        end
        
        function [] = generatemodel(obj)
            
            proneu_info('Generating FloatingBaseSystem model representation.');
            
            % Check the type of multi-rigid-body
            if ~strcmp(obj.robot.description.type, 'floating')
                error('This class only supports FloatingBase systems.');
            else
                generate_system(obj);
            end
            
        end
        
        function [] = simplifydynamics(obj, cmode, ns)
            
            % Check that we all elements are symbols
            if ~isa(obj.symbols.F, 'sym') ||  ~isa(obj.symbols.W, 'sym') || ~isa(obj.symbols.M, 'sym') || ~isa(obj.symbols.f, 'sym')
                error('Simplifications cannot be performed on non-symbolic elements.');
            end
            
            %%% Simplify basic dynamics properties
            
            proneu_info('Performing symbolic simplifications on the system dynamics: ');
            
            proneu_info('-->Simplifying F ');
            tstart = tic;
            obj.F = simplesym(obj.symbols.F, cmode, ns);
            tsimp = toc(tstart);
            proneu_info(['---->Duration: ' num2str(tsimp) ' s']);
            
            proneu_info('-->Simplifying W ');
            tstart = tic;
            obj.W = simplesym(obj.symbols.W, cmode, ns);
            tsimp = toc(tstart);
            proneu_info(['---->Duration: ' num2str(tsimp) ' s']);
            
            proneu_info('-->Simplifying M ');
            tstart = tic;
            obj.M = simplesym(obj.symbols.M, cmode, ns);
            tsimp = toc(tstart);
            proneu_info(['---->Duration: ' num2str(tsimp) ' s']);
            
            proneu_info('-->Simplifying f ');
            tstart = tic;
            obj.f = simplesym(obj.symbols.f, cmode, ns);
            tsimp = toc(tstart);
            proneu_info(['---->Duration: ' num2str(tsimp) ' s']);
            
        end
        
        function [] = generatefunctions(obj, varargin)
            
            proneu_info('Generating numeric functions for the FloatingBaseSystem. ');
            
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
            q = obj.robot.dynamics.symbols.q(:);
            u = obj.robot.dynamics.symbols.u(:);
            tau_a = obj.robot.dynamics.symbols.tau_a(:); 
            tau_e = obj.robot.dynamics.symbols.tau_e(:);
            
            % Store a local copy of the model symbols
            syssymbols = obj.symbols;
            
            % If "values" have been specified then generate non-parametrized functions
            if ~isempty(values)
                syssymbols.F = subs(syssymbols.F, params, values);
                %syssymbols.W = subs(syssymbols.W, params, values);
                syssymbols.M = subs(syssymbols.M, params, values);
                syssymbols.f = subs(syssymbols.f, params, values);
                Fvars = {t, [q; u]};
                %Wvars = {t, [q; u]};
                Mvars = {t, [q; u]};
                fvars = {t, [q; u], tau_a, tau_e};
            else
                Fvars = {t, [q; u], params};
                %Wvars = {t, [q; u], params};
                Mvars = {t, [q; u], params};
                fvars = {t, [q; u], tau_a, tau_e, params};
            end
            
            % Generate matlab functions for each kinematic quantity
            obj.compute.F = matlabFunction(syssymbols.F, 'vars', Fvars);
            %obj.compute.W = matlabFunction(syssymbols.W, 'vars', Wvars);
            obj.compute.M = matlabFunction(syssymbols.M, 'vars', Mvars);
            obj.compute.f = matlabFunction(syssymbols.f, 'vars', fvars);
            
%             % DEBUG
%             obj.compute.tau = matlabFunction(syssymbols.tau, 'vars', fvars);
%             obj.compute.b = matlabFunction(syssymbols.b, 'vars', fvars);
%             obj.compute.g = matlabFunction(syssymbols.g, 'vars', fvars);
            
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
    u = obj.robot.dynamics.symbols.u(:);
    du = obj.robot.dynamics.symbols.du(:);
    
    % Get RobotModel dynamical properties
    M_mbs = obj.robot.dynamics.symbols.M;
    b_mbs = obj.robot.dynamics.symbols.b;
    g_mbs = obj.robot.dynamics.symbols.g;
    tau_mbs = obj.robot.dynamics.symbols.tau;
    F_mbs = obj.robot.dynamics.symbols.F;
    
    % Define the system state vectors
    obj.symbols.q  = q(:);
    obj.symbols.x  = [q(:); u(:)];
    obj.symbols.dx = [dq(:); du(:)];

    % Define the system input vectors
    obj.symbols.u_int = obj.robot.dynamics.symbols.tau_a;
    obj.symbols.u_ext = obj.robot.dynamics.symbols.tau_e;

    % TODO
    obj.symbols.F = F_mbs;
    %obj.symbols.W = [];
    obj.symbols.M = M_mbs;
    obj.symbols.f = tau_mbs - b_mbs - g_mbs;
    
%     % DEBUG
%     obj.symbols.tau = tau_mbs;
%     obj.symbols.b = b_mbs;
%     obj.symbols.g = g_mbs;
    
end
