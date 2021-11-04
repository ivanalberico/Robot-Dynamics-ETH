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

classdef RigidBody < handle
    
    properties 
        % Name of the rigid body element
        name = 'body';
        % Kinematic tree info
        ktree = KinematicTreeNode;
        % Kinematics symbols and comptuations
        kinematics = struct('symbols', RigidBodyKinematics, 'compute', []);
        % Inertial parameters and corresponding numeric values
        inertia = struct('parameters', RigidBodyInertia, 'values', RigidBodyInertia);
        % Geometry specification
        geometry = RigidBodyGeometry;
        % Body parameter specifications
        specs = RigidBodySpecifications;
        % Body graphics for visualization
        graphics = [];
    end
    
    methods
        
        function [ obj ] = RigidBody(varargin)
            
            % Initialize body descriptor
            bodydes = [];
            
            % Check varargin for valid body decription
            if ~isempty(varargin)
                
                % Remove nested cell arguments
                varargin = flatcells(varargin);
                
                % Check for a valid descriptor
                for i=1:numel(varargin)
                    if isa(varargin{i}, 'RigidBodyDescription_v1');
                         bodydes = varargin{i};
                         
                    elseif isa(varargin{i}, 'RigidBodyDescription_v2');
                         bodydes = varargin{i};
                    end
                end
            end
            
            % Throw error if a valid body description has not been defined
            if isempty(bodydes)
                error('Input argument is not of any RigidBodyDescription_<version> type');
            end
            
            % Generate body if valid descriptor has been specified
            bodydes.generatebody(obj);
        end
        
        % Helper to retreive parameter dependencies
        function [ params ] = getparameters(obj)
            
            % Each body holds model and rendering parameters
            params = struct('model', [], 'graphical', []);
            
            % Collect model parameters
            bparams = symvar(obj.kinematics.symbols.T_PiBi);
            params.model = [params.model; bparams(:)];
            bparams = symvar(obj.kinematics.symbols.T_BiCi);
            params.model = [params.model; bparams(:)];
            bparams = symvar(obj.inertia.parameters.m_Bi);
            params.model = [params.model; bparams(:)];
            bparams = symvar(obj.inertia.parameters.Bi_Theta_Bi);
            params.model = [params.model; bparams(:)];
            bparams = symvar(obj.geometry.params);
            params.model = [params.model; bparams(:)];
            
            % Collect graphical parameters
            bparams = symvar(obj.geometry.params);
            params.graphical = [params.graphical; bparams(:)];
            bparams = symvar(sym(obj.geometry.offsets.r));
            params.graphical = [params.graphical; bparams(:)];
            bparams = symvar(sym(obj.geometry.offsets.C));
            params.graphical = [params.graphical; bparams(:)];
            
            % Remove duplciates and ensure column form
            params.model = symvar(params.model);
            params.model = params.model(:);
            params.graphical = symvar(params.graphical);
            params.graphical = params.graphical(:);
        end
        
        % Helper function to create graphics objects for the body visualization
        function [] = creategraphics(obj, varargin)
            
            % Remove nested cell arguments
            varargin = flatcells(varargin);
            
            % Create the body graphics object
            obj.graphics = RigidBodyGraphics(obj.name, varargin);
        end
        
        % Helper function to generate all kinematic computation functions
        function [] = generatefunctions(obj, varargin)
           
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
                error('Variables have not been specified for RigidBody functions.');
            end
            if isempty(parameters)
                error('Parameters have not been specified for RigidBody functions.');
            end
            
            proneu_info(['-->Generating numeric functions for the body element ' obj.name '.']);
            
            % Store a local copy of the kinematics symbols
            rbsymbols = obj.kinematics.symbols;
            
            % If "values" have been specified then generate
            % non-parametrized functions
            if ~isempty(values)
                rbsymbols = numeric_substitutions_minimal(rbsymbols, parameters, values);
                arguments = variables(:);
            else
                arguments = {variables(:), parameters(:)};
                arguments = flatcells(arguments);
            end
            
            % Generate matlab functions for each kinematic quantity
            obj.kinematics.compute = generate_kinematics_functions_minimal(rbsymbols, arguments);
            
        end
        
    end
    
end

%
% HELPER FUNCTIONS
%

function [ symbols ] = numeric_substitutions_minimal(symbols, parameters, values)

    %%% Configuration kinematics
    
    symbols.T_IBi = subs(symbols.T_IBi, parameters, values);
    
    %%% Velocity kinematics
    
    % Angular velocities
    symbols.I_omega_IBi = subs(symbols.I_omega_IBi, parameters, values);
    
    % Linear velocities
    symbols.I_v_IBi = subs(symbols.I_v_IBi, parameters, values);
    
    % Jacobians
    symbols.I_J_IBi = subs(symbols.I_J_IBi, parameters, values);
    
    % Jacobian derivatives
    symbols.I_dJ_IBi = subs(symbols.I_dJ_IBi, parameters, values);
    
end

function [ compute ] = generate_kinematics_functions_minimal(symbols, arguments)

    %%% Configuration kinematics
   
    compute.T_IBi = matlabFunction(symbols.T_IBi, 'vars', arguments);
    
    %%% Velocity kinematics
    
    % Angular velocities
    compute.I_omega_IBi = matlabFunction(symbols.I_omega_IBi, 'vars', arguments);
    
    % Linear velocities
    compute.I_v_IBi = matlabFunction(symbols.I_v_IBi, 'vars', arguments);
    
    % Jacobians
    compute.I_J_IBi = matlabFunction(symbols.I_J_IBi, 'vars', arguments);
    
    % Jacobian derivatives
    compute.I_dJ_IBi = matlabFunction(symbols.I_dJ_IBi, 'vars', arguments);
    
end
