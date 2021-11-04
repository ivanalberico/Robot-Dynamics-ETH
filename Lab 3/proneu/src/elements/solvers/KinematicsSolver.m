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

classdef KinematicsSolver < handle
    
    % Solver interface
    properties (Access = public)
        % Model functions
        system = [];
        % Excitation functions
        input = struct('int',[]);
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
            
            proneu_info('Setting up KinematicsSolver instance.');
            
            % Set the system functions
            obj.input.int = uifunc;
            
            % Set parameter values
            obj.parameters = parameters;
            
       end
        
        function [data, time, tcomp] = compute(obj, xinit, tstart, tstop, tstep)

            % Start compuation timer
            cstart = tic;

            % Set solver time configurations
            Nt = uint64((tstop - tstart)/tstep);
            Nx = size(xinit,1);
            Nq = ceil(Nx/2);
            
            % Initialize data structs
            time = linspace(tstart, tstop, Nt);
            data = zeros(Nx,Nt);
            
            % Initialize iterators
            q_k = xinit(1:Nq);
            
            % Integrate system over specified time interval
            for k=1:Nt
                
                % Set current time
                t_k = tstart + double(k-1)*tstep;
               
                % System velocities specified by kinematics controller
                dq_k = obj.input.int(t_k, q_k);
                
                % Integrate system velocities
                q_k = q_k + tstep * dq_k;
                
                % Store result
                data(:,k) = [q_k; dq_k];
                
            end

            % Terminate computation timer
            tcomp = toc(cstart);
        end
        
    end
    
end

%
% HELPER FUNCTIONS
%
