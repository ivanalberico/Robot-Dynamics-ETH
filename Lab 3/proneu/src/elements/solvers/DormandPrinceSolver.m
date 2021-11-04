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

classdef DormandPrinceSolver < handle
    
    % Solver interface
    properties (Access = public)
        % Model functions
        system = struct('M', [], 'f', []);
        % Excitation functions
        input = struct('int', [], 'ext', []);
        % Solver options
        options = []; 
    end
    
    % Solver internals
    properties (Access = private)
        % System dynamics compute function
        systfunc = []; 
        % System mass-matrix compute function
        massfunc = [];
    end
    
    % Solver operations
    methods
        
        function [] = setup(obj, Nx, Nq, sfunc, uifunc, uefunc, parameters, varargin)
            
            proneu_info('Setting up DormandPrinceSolver instance.');
            
            % Initialize auxiliary callbacks
            mmclbk = [];
            sfclbk = [];
            
            % Parse arguments
            if ~isempty(varargin)
                for i=1:numel(varargin)
                    if ischar(varargin{i}) && strcmp(varargin{i}, 'massclbk')
                        mmclbk = varargin{i};
                    elseif ischar(varargin{i}) && strcmp(varargin{i}, 'sysclbk')
                        sfclbk = varargin{i};
                    end    
                end
            end
            
            % Set the system dynamics function
            obj.system.M = sfunc.M;
            obj.system.f = sfunc.f;
            obj.input.int = uifunc;
            obj.input.ext = uefunc;
            
            % Set the dynamics computation wrapper
            obj.systfunc = @(t,y) runsystfunc(sfclbk,obj.system.f,t,y,parameters, obj.input.int(t,y), obj.input.ext(t,y));
            
            % Set the mass-matrix computation wrapper
            obj.massfunc = @(t,y) runmassfunc(mmclbk,obj.system.M,t,y,parameters);
            obj.options = odeset('Mass', obj.massfunc, 'OutputFcn',[]);
            
        end
        
        function [data, time, tcomp] = compute(obj, init, tstart, tstop, tstep)
            
            % Set solver time configurations
            N = uint64((tstop - tstart)/tstep);
            time = linspace(tstart, tstop, N);
            tspan = [tstart tstop];

            % Start compuation timer
            cstart = tic;

            % Set solver to standard ode45
            ode_sol = ode45(obj.systfunc, tspan, init.', obj.options);

            % Extend time if solver fails before tstop
            Nsol = size(ode_sol.x,2);
            if ode_sol.x(Nsol) < time(N)
                time = linspace(ode_sol.x(1), ode_sol.x(Nsol), N);
            end

            % Evaluate data at intermediate uniform points
            data = deval(ode_sol, time);

            % Terminate computation timer
            tcomp = toc(cstart);
        end
        
    end
    
end

%
% HELPER FUNCTIONS
%

function expr = runsystfunc(extcallback,systfunc,t,state,params,u_a,u_e)

    % Execute the ODE dynamics implementation
    expr = systfunc(t,state,u_a,u_e,params);
    
    % If the callback handle is not empy, execute the callback operation.
    if ~isempty(extcallback)
        extcallback(expr,t,state,params,u_a,u_e);
    end
    
end

function expr = runmassfunc(extcallback,massfunc,t,state,params)

    % Execute the ODE dynamics implementation
    expr = massfunc(t,state,params);
    
    % If the callback handle is not empy, execute the callback operation.
    if ~isempty(extcallback)
        extcallback(expr,t,state,params);
    end
    
end
