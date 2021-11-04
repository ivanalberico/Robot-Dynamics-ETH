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

%
% What do we want to monitor?:
%
% Fixed & Floating:
%
%   * Joint positions
%   * Joint velocities
%   * Joint torques
% 
%   * End-effector pose.
%   * End-effector velocities.
%   * End-effector wrench
% 
% Floating Only:
%
%   * Base pose
%   * Base velocities
%   * Base wrench
%

classdef DataScope < handle
    
    % Class interface data
    properties (Access = public)
        %
        name = 'DataScope';
        %
        scfigure = [];
        %
        scaxes = [];
    end
    
    % Class internals
    properties (Access = private)
        % Internal handle to a RobotModel object
        robot = [];
        % Internal handle to End-Effector RibidBody types
        endeffector = [];
    end
    
    % Class interface operations
    methods
        
        function [ obj ] = DataScope(model, varargin)
            % Check if name has been specified.
            if ~isempty(model)
                obj.robot = model;
            else
                error('Invalid input. Argument must be of type "RobotModel".');
            end
        end
        
        function [] = open(obj)
            
            % Set the figure name
            scfigurename = strcat(obj.name,':: ', obj.robot.name);
            scfontsize = 12; % TODO
            
            % Generate new figure and axes objects
            obj.scfigure = figure('Name', scfigurename, 'NumberTitle', 'off');
            
            % Generate subplots
            obj.scaxes = cell(6,1);
            for k=1:6
                obj.scaxes{k} = axes('parent', obj.scfigure);
                subplot(2,3,k,obj.scaxes{k});
                set(obj.scaxes{k},'TickLabelInterpreter','latex','FontSize',scfontsize);
                title('Data','Interpreter','latex','FontSize',scfontsize)
                xlabel('time [s]','Interpreter','latex','FontSize',scfontsize);
                ylabel('data [units]','Interpreter','latex','FontSize',scfontsize);
                grid on;
            end
            
            %
            %
            %
            
        end
        
        function [] = close(obj)
            %
            % TODO
            %
        end
        
        function [] = save(obj)
            %
            % TODO
            %
        end
        
        function [] = export(obj)
            %
            % TODO
            %
        end
        
    end
    
end

%
% HELPER FUNCTIONS
%

% function zoom_callback(~, evd)
%     if evd.Axes == simviz_3d_axes
%         axis_limits = axis(evd.Axes);
%         for k = 1 : Np+1
%             if csfLabelsActive{k} == true
%                 set(csflabels{k},'FontSize',100*csscale/(axis_limits(4)-axis_limits(3)));
%             end
%         end
%     end
% end

% % Determine the representation from the type of system
% switch (obj.robot.description.type)
%     case 'fixed'
%         isfloating = false;
%         x = obj.robot.dynamics.symbols.q;
%         dx = obj.robot.dynamics.symbols.dq;
%         ddx = obj.robot.dynamics.symbols.ddq;
% 
%     case 'floating'
%         isfloating = true;
%         x = obj.robot.dynamics.symbols.q;
%         dx = obj.robot.dynamics.symbols.u;
%         ddx = obj.robot.dynamics.symbols.du;
% end
% 
% % Set the system inputs 
% tau_jnt = obj.robot.dynamics.symbols.tau_a;
% tau_env = obj.robot.dynamics.symbols.tau_e;
% 
% % Set the system quantity lenths
% Nrb = length(obj.robot.body);
% Nx = length(x);
% Ndx = length(dx);
% Nddx = length(ddx);
% Ntauj = length(tau_jnt);
% Ntauenv = length(tau_env);
% 
