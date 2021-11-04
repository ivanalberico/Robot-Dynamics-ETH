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

classdef FixedStepFloatingBaseSolver < handle
    
    % Solver interface
    properties (Access = public)
        % Model functions
        system = struct('F', [], 'W', [],'M', [], 'f', []);
        % Excitation functions
        input = struct('int', [], 'ext', []);
        % Solver options
        options = []; 
    end
    
    % Solver internals
    properties (Access = private)
        % Number of state variables
        Nx = [];
        % Number of generalized coordinate variables
        Nq = [];
        % System model parameter values
        parameters;
    end
    
    % Solver operations
    methods
        
        function [] = setup(obj, Nx, Nq, sfunc, uifunc, uefunc, parameters, varargin)
            
            proneu_info('Setting up FixedStepFloatingBaseSolver instance.');
            
            % Set system model function
            obj.system.F = sfunc.F;
            %obj.system.W = sfunc.W;
            obj.system.M = sfunc.M;
            obj.system.f = sfunc.f;
            
            % Set internal actuator controller function
            obj.input.int = uifunc;
            
            % Set external environment function
            obj.input.ext = uefunc;
            
            % Set model parameter values
            obj.parameters = parameters;
            
            % Set system dimensions
            obj.Nx = Nx;
            obj.Nq = Nq;
            
        end
        
        function [data, time, tcomp] = compute(obj, xinit, tstart, tstop, tstep)
            
            % Start compuation timer
            cstart = tic;
            
            % Set solver time configurations
            Nt = uint64((tstop - tstart)/tstep);
            time = linspace(tstart, tstop, Nt);
            data = zeros(obj.Nx,Nt);
            
            % Initialize iterators
            x_k = xinit;
            q_k = xinit(1:obj.Nq);
            u_k = xinit(obj.Nq+1:end);
            
            for k=1:Nt
                
                % Set current time
                t_k = tstart + double(k-1)*tstep;
                
                % Compute internal and environmental influences
                tau_a_k = obj.input.int(t_k, x_k);
                tau_e_k = obj.input.ext(t_k, x_k);
                
                % Compute dynamics and kinematics at current step
                F_k = obj.system.F(t_k, x_k, obj.parameters);
                M_k = obj.system.M(t_k, x_k, obj.parameters);
                f_k = obj.system.f(t_k, x_k, tau_a_k, tau_e_k, obj.parameters);
                
                % Integrate system accelerations
                du_k = M_k\f_k;
                u_k = u_k + tstep * du_k;
                
                % Integrate coordinate velocities
                dq_k = F_k * u_k;
                q_k = q_k + tstep * dq_k;
                
                % Store result
                x_k = [q_k; u_k];
                data(:,k) = x_k;
                
%                 % DEBUG
%                 tau_k = obj.system.tau(t_k, x_k, tau_a_k, tau_e_k, obj.parameters);
%                 b_k = obj.system.b(t_k, x_k, tau_a_k, tau_e_k, obj.parameters);
%                 g_k = obj.system.g(t_k, x_k, tau_a_k, tau_e_k, obj.parameters);
%                 disp(' ');
%                 disp('DATA: ');
%                 disp(['f   = ' num2str(f_k.')]);
%                 disp(['tau = ' num2str(tau_k.')]);
%                 disp(['b   = ' num2str(b_k.')]);
%                 disp(['g   = ' num2str(g_k.')]);
%                 disp(['du  = ' num2str(du_k.')]);
%                 disp(['u   = ' num2str(u_k.')]);
%                 disp(['dq  = ' num2str(dq_k.')]);
%                 disp(['q   = ' num2str(q_k.')]);
                
            end
            
            % Terminate computation timer
            tcomp = toc(cstart);
            
        end
        
    end
    
end