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

classdef FloatingBaseDynamics < handle
    
    % Floating-Base System definitions
    properties
        q       % Generalized coordinates
        dq      % Generalized velocities
        ddq     % Generalized accelerations
        u       % Generalized quasi-velocities vector used for floating-base systems
        du      % Generalized quasi-accelerations vector used for floating-base systems
        F       % Matrix to map u to to dq
        dF      % Matrix to map {u, du} to ddq
        G       % Matrix to map dq to to u
        dG      % Matrix to map {dq,ddq} to to du
    end
    
    % Dynamical properties
    properties
        M       % Generalized mass matrix
        b       % Generalized non-linear Coriolis and centrifugal forces
        g       % Generalized gravitational forces
        tau     % Total generalized non conservative forces
        tau_act % Generalized forces induced by actuator forces and moments
        tau_ext % Generalized forces induced by external forces and moments
        S_a     % Selection matrix for the actuation forces
        tau_a   % Vector of actuator forces/torques
        S_e     % Selection matrix for the external forces
        tau_e   % Vector of external forces/torques
    end
    
    % Energy-based scalar properties
    properties
        T % Kinetic energy
        U % Potential energy
        L % Lagrangian function
        H % Hamiltonian function
    end
    
    % Dynamics formulations
    properties
        invM    % Inverse generalized mass matrix
        FD      % Forward dynamics
        ID      % Inverse dynamics
    end
    
    % Dynamics computations
    methods
        
        function [] = getenergies(obj)
            
            proneu_info('Computing the energy-based dynamics quantities: ');
            
            % Compute the total kinetic energy of the system
            proneu_info('-->Computing kinetic energy T ');
            tstart = tic;
            obj.T = obj.u.' * obj.M * obj.u;
            tcomp = toc(tstart);
            proneu_info(['---->Duration: ' num2str(tcomp) ' s']);
            
            % Compute the total potential energy of the system
            proneu_info('-->Computing potential energy U ');
            tstart = tic;
            obj.U = sym(0);
            tcomp = toc(tstart);
            proneu_info(['---->Duration: ' num2str(tcomp) ' s']);
            
            % Compute the total kinetic energy of the system
            proneu_info('-->Computing the Lagrangian L ');
            tstart = tic;
            obj.L = obj. T - obj.U;
            tcomp = toc(tstart);
            proneu_info(['---->Duration: ' num2str(tcomp) ' s']);
            
            % Compute the total kinetic energy of the system
            proneu_info('-->Computing the Hamiltonian H ');
            tstart = tic;
            obj.H = legendre(obj.dq, obj.L);
            tcomp = toc(tstart);
            proneu_info(['---->Duration: ' num2str(tcomp) ' s']);
            
        end
        
        function [] = getinversemassmatrix(obj)
            
            proneu_info('Computing the inverse of the symbolic mass matrix M. ');
            
            % Compute the total equations of motion as an implicit function
            tstart = tic;
            obj.invM = inv(obj.M);
            tcomp = toc(tstart);
            
            % Display total computation time
            proneu_info(['-->Duration: ' num2str(tcomp) ' s']);
        end
        
        function [] = getforwarddynamics(obj)
            % Compute the total equations of motion as an implicit function
            if ~isempty(obj.invM)
                
                proneu_info('Computing the Forward Dynamics.');
                
                % Compute the total equations of motion as an implicit function
                tstart = tic;
                obj.FD = obj.invM * (obj.tau - obj.b - obj.g);
                tcomp = toc(tstart);

                % Display total computation time
                proneu_info(['-->Duration: ' num2str(tcomp) ' s']);
                
            else
                error('Cannot compute forward dynamics without the inverse mass matrix.');
            end
        end
        
        function [] = getinversedynamics(obj)
            
            proneu_info('Computing the Inverse Dynamics.');

            % Compute the total equations of motion as an implicit function
            tstart = tic;
            obj.ID = obj.M * obj.du + obj.b + obj.g - obj.tau_ext;
            tcomp = toc(tstart);

            % Display total computation time
            proneu_info(['-->Duration: ' num2str(tcomp) ' s']);
            
        end
        
        function [] = simplifydynamics(obj, cmode, ns)
            
            % Check that we all elements are symbols
            if ~isa(obj.M, 'sym') || ~isa(obj.b, 'sym') || ~isa(obj.g, 'sym') || ~isa(obj.tau, 'sym')
                error('Simplifications cannot be performed on non-symbolic elements.');
            end
            
            proneu_info('Performing symbolic simplifications on the dynamics quantities: ');
            
            %%% Simplify basic dynamics properties
            
            proneu_info('-->Simplifying M ');
            tstart = tic;
            obj.M = simplesym(obj.M, cmode, ns);
            tsimp = toc(tstart);
            proneu_info(['---->Duration: ' num2str(tsimp) ' s']);
            
            proneu_info('-->Simplifying b ');
            tstart = tic;
            obj.b = simplesym(obj.b, cmode, ns);
            tsimp = toc(tstart);
            proneu_info(['---->Duration: ' num2str(tsimp) ' s']);
            
            proneu_info('-->Simplifying g ');
            tstart = tic;
            obj.g = simplesym(obj.g, cmode, ns);
            tsimp = toc(tstart);
            proneu_info(['---->Duration: ' num2str(tsimp) ' s']);
            
            proneu_info('-->Simplifying tau ');
            tstart = tic;
            obj.tau = simplesym(obj.tau, cmode, ns);
            tsimp = toc(tstart);
            proneu_info(['---->Duration: ' num2str(tsimp) ' s']);
            
            %%% Simplify energy-based properties
            
            if ~isempty(obj.T)
                proneu_info('-->Simplifying T ');
                tstart = tic;
                obj.T = simplesym(obj.T, cmode, ns);
                tsimp = toc(tstart);
                proneu_info(['---->Duration: ' num2str(tsimp) ' s']);
            end
            
            if ~isempty(obj.U)
                proneu_info('-->Simplifying U ');
                tstart = tic;
                obj.U = simplesym(obj.U, cmode, ns);
                tsimp = toc(tstart);
                proneu_info(['---->Duration: ' num2str(tsimp) ' s']);
            end
            
            if ~isempty(obj.L)
                proneu_info('-->Simplifying L ');
                tstart = tic;
                obj.L = simplesym(obj.L, cmode, ns);
                tsimp = toc(tstart);
                proneu_info(['---->Duration: ' num2str(tsimp) ' s']);
            end
            
            if ~isempty(obj.H)
                proneu_info('-->Simplifying H ');
                tstart = tic;
                obj.H = simplesym(obj.H, cmode, ns);
                tsimp = toc(tstart);
                proneu_info(['---->Duration: ' num2str(tsimp) ' s']);
            end
            
            %%% Simplify total dynamics properties
            
            if ~isempty(obj.invM)
                proneu_info('-->Simplifying the inverse mass matrix ');
                tstart = tic;
                obj.invM = simplesym(obj.invM, cmode, ns);
                tsimp = toc(tstart);
                proneu_info(['---->Duration: ' num2str(tsimp) ' s']);
            end
            
            if ~isempty(obj.FD)
                proneu_info('-->Simplifying the forward dynamics ');
                tstart = tic;
                obj.FD = simplesym(obj.FD, cmode, ns);
                tsimp = toc(tstart);
                proneu_info(['---->Duration: ' num2str(tsimp) ' s']);
            end
            
            if ~isempty(obj.ID)
                proneu_info('-->Simplifying the inverse dynamics ');
                tstart = tic;
                obj.ID = simplesym(obj.ID, cmode, ns);
                tsimp = toc(tstart);
                proneu_info(['---->Duration: ' num2str(tsimp) ' s']);
            end
            
        end
        
        function [ funcs ] = generatefunctions(obj, varargin)
            
            % Initialize settings
            variables = [];
            parameters = [];
            values = [];
            mode = 'minimal';
            
            % Check arguments
            if ~isempty(varargin)
                for i=1:numel(varargin)
                    if ischar(varargin{i}) && strcmp(varargin{i}, 'variables')
                        variables = varargin{i+1};
                    elseif ischar(varargin{i}) && strcmp(varargin{i}, 'parameters')
                        parameters = varargin{i+1};
                    elseif ischar(varargin{i}) && strcmp(varargin{i}, 'values')
                        values = varargin{i+1};
                    elseif ischar(varargin{i}) && strcmp(varargin{i}, 'mode')
                        mode = varargin{i+1};
                    end    
                end
            end
            
            % Check for errors
            if isempty(variables)
                error('Variables have not been specified for FloatingBodyDynamics functions.');
            end
            if isempty(parameters)
                error('Parameters have not been specified for FloatingBodyDynamics functions.');
            end
            
            proneu_info('-->Generating numeric functions for the Floating-Base dynamics. ');
            
            % Store a local copy of the kinematics symbols
            dynsymbols = copy_symbols(obj);
            
            % If "values" have been specified then generate
            % non-parametrized functions
            if ~isempty(values)
                dynsymbols = numeric_substitutions(dynsymbols, parameters, values);
                arguments = variables(:);
            else
                arguments = {variables(:), parameters(:)};
                arguments = flatcells(arguments);
            end
            
            % Generate matlab functions for each kinematic quantity
            tstart = tic;
            funcs = generate_functions(dynsymbols, arguments);
            tsimp = toc(tstart);
            proneu_info(['---->Duration: ' num2str(tsimp) ' s']);
            
        end
        
    end
end

%
% HELPER FUNCTIONS
%

function [ symbols ] = copy_symbols(obj)

    % Generalized velocity and acceleration maps
    
    symbols.F   = obj.F;
    symbols.dF  = obj.dF;
%     symbols.G   = obj.G;
%     symbols.dG  = obj.dG;

    % Dynamical properties
    
    symbols.M       = obj.M;
    symbols.b       = obj.b;
    symbols.g       = obj.g;
    symbols.tau     = obj.tau;
    
    symbols.tau_act = obj.tau_act;
    symbols.tau_ext = obj.tau_ext;
    symbols.S_a     = obj.S_a;
    symbols.S_e     = obj.S_e;

    % Energy properties
    
    symbols.T = obj.T;
    symbols.U = obj.U;
    symbols.L = obj.L;
    symbols.H = obj.H;

    % Dynamics formulations
    
    symbols.invM    = obj.invM;
    symbols.FD      = obj.FD;

end

function [ symbols ] = numeric_substitutions(symbols, parameters, values)

    % Generalized velocity and acceleration maps
    
    symbols.F   = subs(symbols.F, parameters, values);
    symbols.dF  = subs(symbols.dF, parameters, values);
%     symbols.G   = subs(symbols.G, parameters, values);
%     symbols.dG  = subs(symbols.dG, parameters, values);

    % Dynamical properties
    
    symbols.M       = subs(symbols.M, parameters, values);
    symbols.b       = subs(symbols.b, parameters, values);
    symbols.g       = subs(symbols.g, parameters, values);
    symbols.tau     = subs(symbols.tau, parameters, values);
    
    symbols.tau_act = subs(symbols.tau_act, parameters, values);
    symbols.tau_ext = subs(symbols.tau_ext, parameters, values);
    symbols.S_a     = subs(symbols.S_a, parameters, values);
    symbols.S_e     = subs(symbols.S_e, parameters, values);

    % Energy properties
    
    if ~isempty(symbols.T)
        symbols.T = subs(symbols.T, parameters, values);
    end
    
    if ~isempty(symbols.U)
        symbols.U = subs(symbols.U, parameters, values);
    end
    
    if ~isempty(symbols.L)
        symbols.L = subs(symbols.L, parameters, values);
    end
    
    if ~isempty(symbols.H)
        symbols.H = subs(symbols.H, parameters, values);
    end
    
    % Dynamics formulations
    
    if ~isempty(symbols.invM)
        symbols.invM    = subs(symbols.invM, parameters, values);
    end
    
    if ~isempty(symbols.FD)
        symbols.FD      = subs(symbols.FD, parameters, values);
    end

end

function [ compute ] = generate_functions(symbols, arguments)
    
    % Generalized velocity and acceleration maps
    
    compute.F  = matlabFunction(symbols.F, 'vars', arguments);
    compute.dF = matlabFunction(symbols.dF, 'vars', arguments);
%     compute.G  = matlabFunction(symbols.G, 'vars', arguments);
%     compute.dG = matlabFunction(symbols.dG, 'vars', arguments);

    % Dynamical properties

    compute.M       = matlabFunction(symbols.M, 'vars', arguments);
    compute.b       = matlabFunction(symbols.b, 'vars', arguments);
    compute.g       = matlabFunction(symbols.g, 'vars', arguments);
    compute.tau     = matlabFunction(symbols.tau, 'vars', arguments);
    
    compute.tau_act = matlabFunction(symbols.tau_act, 'vars', arguments);
    compute.tau_ext = matlabFunction(symbols.tau_ext, 'vars', arguments);
    compute.S_a     = matlabFunction(symbols.S_a, 'vars', arguments);
    compute.S_e     = matlabFunction(symbols.S_e, 'vars', arguments);

    % Energy properties
    
    if ~isempty(symbols.T)
        compute.T = matlabFunction(symbols.T, 'vars', arguments);
    end
    
    if ~isempty(symbols.U)
        compute.U = matlabFunction(symbols.U, 'vars', arguments);
    end
    
    if ~isempty(symbols.L)
        compute.L = matlabFunction(symbols.L, 'vars', arguments);
    end
    
    if ~isempty(symbols.H)
        compute.H = matlabFunction(symbols.H, 'vars', arguments);
    end
    
    % Dynamics formulations
    
    if ~isempty(symbols.invM)
        compute.invM = matlabFunction(symbols.invM, 'vars', arguments);
    end
    
    if ~isempty(symbols.FD)
        compute.FD = matlabFunction(symbols.FD, 'vars', arguments);
    end

end
