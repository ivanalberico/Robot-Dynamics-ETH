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

classdef FixedStepSolver < handle
    
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
        % System model parameters
        parameters = [];
    end
    
    % Solver operations
    methods
        
        function [] = setup(obj, Nx, Nq, sfunc, uifunc, uefunc, parameters, varargin)
            
            proneu_info('Setting up FixedStepSolver instance.');
            
            % Set the system dynamics function
            obj.system.M = sfunc.M;
            obj.system.f = sfunc.f;
            obj.input.int = uifunc;
            obj.input.ext = uefunc;
            
            % Set parameter values
            obj.parameters = parameters;
            
       end
        
        function [data, time, tcomp] = compute(obj, xinit, tstart, tstop, tstep)

            % Start compuation timer
            cstart = tic;

            % Set solver time configurations
            Nt = uint64((tstop - tstart)/tstep);
            time = linspace(tstart, tstop, Nt);
            data = zeros(size(xinit,1),Nt);
            
            % Initialize iterators
            x_k = xinit;
            
            for k=1:Nt
                
                % Set current time
                t_k = tstart + double(k-1)*tstep;
                
                % Compute internal and environmental influences
                tau_a_k = obj.input.int(t_k, x_k);
                tau_e_k = obj.input.ext(t_k, x_k);
                
                % Compute dynamics and kinematics at current step
                M_k = obj.system.M(t_k, x_k, obj.parameters);
                f_k = obj.system.f(t_k, x_k, tau_a_k, tau_e_k, obj.parameters);
                
                % Integrate system accelerations
                dx_k = M_k\f_k;
                x_k = x_k + tstep * dx_k;
                
                % Store result
                data(:,k) = x_k;
                
            end

            % Terminate computation timer
            tcomp = toc(cstart);
        end
        
    end
    
end

%
% HELPER FUNCTIONS
%
