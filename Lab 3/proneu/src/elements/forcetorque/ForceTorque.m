classdef ForceTorque < handle
    properties
        % Name of force-torque object
        name = 'forcetorque';
        % Kinematics tree meta-data
        ktree = KinematicTreeEdge;
        % Type of force-torque element: 'linear', 'rotational', 'wrench'
        type = 'none';
        % Symbolic expressions for the physical quantities
        symbols = ForceTorqueProperties;
        % Matlab functions for numerical computations
        compute = [];
        % Graphics object for visualization
        graphics = [];
    end
    
    methods
       
        function [ obj ] = ForceTorque(varargin)
            
            % Initialize element descriptor
            ftdes = [];
            
            % Check varargin for valid body decription
            if ~isempty(varargin)
                
                % Flatten nested cell arrays
                varargin = flatcells(varargin);
                
                % Check for a valid descriptor
                for i=1:numel(varargin)
                    if isa(varargin{i}, 'ForceTorqueDescription_v1');
                         ftdes = varargin{i};
                    
                    elseif isa(varargin{i}, 'ForceTorqueDescription_v2');
                         ftdes = varargin{i};
                    end
                end
            end
            
            % Throw error if a valid body description has not been defined
            if isempty(ftdes)
                error('Input argument is not of any ForceTorqueDescription_<version> type');
            end
            
            % Generate force-torque element if valid descriptor has been specified
            ftdes.generateforcetorque(obj);
        end
        
        function [ params ] = getparameters(obj)
            
            % Each body holds model and rendering parameters
            params = struct('model', [], 'graphical', []);
            
            % Collect model parameters
            bparams = symvar(obj.symbols.Pi_r_PiRj);
            params.model = [params.model; bparams(:)];
            bparams = symvar(obj.symbols.Bi_r_BiAj);
            params.model = [params.model; bparams(:)];
            bparams = symvar(obj.symbols.Bi_F_j);
            params.model = [params.model; bparams(:)];
            bparams = symvar(obj.symbols.Bi_T_j);
            params.model = [params.model; bparams(:)];
            
            % Remove duplciates and ensure column form
            params.model = symvar(params.model);
            params.model = params.model(:);
            
        end
        
        % Helper function to create graphics objects for the body visualization
        function [] = creategraphics(obj, varargin)
            
            % Remove nested cell arguments
            varargin = flatcells(varargin);
            
            % Create the body graphics object
%             obj.graphics = ForceTorqueGraphics(obj.name, varargin);
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
                error('Variables have not been specified for ForceTorque functions.');
            end
            if isempty(parameters)
                error('Parameters have not been specified for ForceTorque functions.');
            end
            
            proneu_info(['-->Generating numeric functions for the force-torque element ' obj.name '.']);
            
            % Store a local copy of the kinematics symbols
            ftsymbols = obj.symbols;
            
            % If "values" have been specified then generate
            % non-parametrized functions
            if ~isempty(values)
                ftsymbols = numeric_substitutions(ftsymbols, parameters, values);
                arguments = variables(:);
            else
                arguments = {variables(:), parameters(:)};
                arguments = flatcells(arguments);
            end
            
            % Generate matlab functions for each kinematic quantity
            obj.compute = generate_functions(ftsymbols, arguments);
            
        end
    end
    
end

%
% HELPER FUNCTIONS
%

function [ symbols ] = numeric_substitutions(symbols, parameters, values)
    
    % Substitute the numeric parameters into the defining quantities
    symbols.I_F_j   = subs(symbols.I_F_j, parameters, values);
    symbols.I_T_j   = subs(symbols.I_T_j, parameters, values);
  
    % Substitute the numeric parameters into the derived quantities
    symbols.I_J_j   = subs(symbols.I_J_j, parameters, values);
    symbols.tau_j   = subs(symbols.tau_j, parameters, values);
    
end

function [ compute ] = generate_functions(symbols, arguments)

    % Generate numeric functions for the defining quantities
    compute.I_F_j   = matlabFunction(symbols.I_F_j, 'vars', arguments);
    compute.I_T_j   = matlabFunction(symbols.I_T_j, 'vars', arguments);
  
    % Generate numeric functions for the defining quantities
    compute.I_J_j   = matlabFunction(symbols.I_J_j, 'vars', arguments);
    compute.tau_j   = matlabFunction(symbols.tau_j, 'vars', arguments);

end
