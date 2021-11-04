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

classdef RobotModel < handle
    
    % Model description
    properties (Access = public)
        % Name of the model
        name = 'model';
        % Description of the system dynamics
        description = struct('type', [], ...
                             'method', [], ...
                             'symsteps', [], ...
                             'orientation', [], ...
                             'kinematictree', [], ...
                             'massmatrixsparisty', []);
    end
    
    % Model elements
    properties (Access = public)
        % Array of RigidBody elements
        body = RigidBody.empty;
        % Array of ForceTorque elements
        force = ForceTorque.empty;
    end
    
    % Generated model quantities
    properties (Access = public)
        % Elements of the system dynamics
        dynamics    = struct('symbols', [], 'compute', []);
        % Stuct of arrays for the system parameter symbols and values
        parameters  = struct('symbols', [], 'values', []);
    end
    
    % Class internals
    properties (Access = private)
        % Internal store of model arguments
        modelargs = struct('q', [], ...
                           'tau_a', [], ...
                           'tau_e', [], ...
                           'I_a_g', []);
        % Computation time measurements
        comptime = struct('trbgen', 0, ...
                          'tftgen', 0, ...
                          'tsysdef', 0, ...
                          'tmodelgen', 0, ...
                          'tparamcomp', 0, ...
                          'total', 0);
    end
    
    % Class operations
    methods
        
        function [ obj, comptime ] = RobotModel(bodies, forces, q, tau_a, tau_e, I_a_g, varargin)
            
            %%%
            % Preamble
            %%%
            
            % Initial greeting
            proneulogo();
            proneu_info('Starting proNEu symbolic engine...');
            
            % Check if type and method have been defined by user
            proneu_info('Processing and checking arguments...');
            check_arguments(obj, q, tau_a, tau_e, I_a_g, varargin);
            
            % Start performance counter
            comptime = obj.comptime;
            
            % System specific output
            proneu_info(['Generating a RobotModel for system named: "' obj.name '"']);
            
            %%%
            % Generate the body elements
            %%%
            
            proneu_info('Creating RigidBody objects from body specifcations.');
            Nb = numel(bodies);
            tstart = tic;
            for i=1:Nb
                obj.body(i) = RigidBody(bodies(i));
            end
            comptime.trbgen = toc(tstart);
            
            %%%
            % Generate the force-torque elements
            %%%
            
            proneu_info('Creating ForceTorque objects from force specifcations.');
            
            Nft = numel(forces);
            tstart = tic;
            for j=1:Nft
                obj.force(j) = ForceTorque(forces(j));
            end
            comptime.tftgen = toc(tstart);
            
            %%%
            % Kinematic tree generation
            %%%
            
            proneu_info('Generating kinematics tree for specified system.');
            
            %
            % TODO
            %
            
            %%%
            % Generate System Definitions
            %%%
            
            if ~strcmp(obj.description.type, 'none')
                % Generate the system definitions
                proneu_info('Creating multi-body system definitions.');
                tstart = tic;
                define_multibody_system(obj, q, varargin);
                comptime.tsysdef = toc(tstart);
            end
            
            %%%
            % Generate Dynamics
            %%%
            
            if ~strcmp(obj.description.method, 'none')
                % Generate the model of the system
                proneu_info(['Generating symbolic kinematics and dynamics using method: "' obj.description.method '".']);
                tstart = tic;
                generate_model(obj);
                comptime.tmodelgen = toc(tstart);
            else
                proneu_info('Generating robot model containers.');
                comptime.tmodelgen = 0;
            end
            
            %%%
            % Generate model parameter definitions
            %%%
            
            proneu_info('Collecting model parameters.');
            tstart = tic;
            collect_model_parameters(obj);
            comptime.tparamcomp = toc(tstart);
            
            %%%
            % Results
            %%%
            
            % Store total model generation time and display results
            comptime.total = comptime.trbgen + comptime.tftgen + comptime.tsysdef + comptime.tmodelgen + comptime.tparamcomp;
            obj.comptime = comptime;
            proneu_info(['Total RobotModel generation time is : ' num2str(comptime.total) ' s']);
            
        end
        
        function [ ktree, ktfig ] = viewkinematictree(obj)
            %
            % TODO
            %
        end
        
        function [ spmatrix, spfig ] = massmatrixsparsity(obj)
            
            % Get dimenions of mass matrix
            mmdim = size(obj.dynamics.symbols.M);
            
            % Initialize mass matrix sparisty map
            spmatrix = zeros(mmdim(1), mmdim(2));
            
            % Generate the mass matrix sparisty pattern
            for mi=1:mmdim(1)
                for mj=1:mmdim(2)
                    if obj.dynamics.symbols.M(mi,mj) ~= sym(0)
                        spmatrix(mi,mj) = 1;    
                    end
                end
            end
            
            % Store sparsity pattern internally
            obj.description.massmatrixsparsity = spmatrix;
            
            % Use matlab tools to plot sparsity pattern
            spfig = figure('Name','Mass-Matrix Sparsity Pattern', 'NumberTitle', 'off', 'WindowStyle', 'docked');
            spy(obj.dynamics.symbols.M);
        end
        
        function [] = generatemodel(obj)
            
            proneu_info(['Generating symbolic kinematics and dynamics using method"' obj.description.method '".']);
            
            % Generate the model of the system
            tstart = tic;
            generate_model(obj);
            obj.comptime.tmodelgen = toc(tstart);
            
            proneu_info(['Total model generation time is : ' num2str(obj.comptime.tmodelgen) ' s']);
            
        end
        
        function [] = generatefunctions(obj, varargin)
            
            % Initialise to parametrized functions
            isparametrized = true;  % true, false
            paramvalues = [];       % numeric array of values
            mode = 'min';           % 'min', 'ext', 'full', 'all'
            
            % Determine if the functions should be parametrized or
            % hard-coded with current parameter values
            if ~isempty(varargin)
                for i=1:numel(varargin)
                    if ischar(varargin{i}) && strcmp(varargin{i}, 'parametrized')
                        isparametrized = varargin{i+1};
                    elseif ischar(varargin{i}) && strcmp(varargin{i}, 'values')
                        paramvalues = varargin{i+1};
                    elseif ischar(varargin{i}) && strcmp(varargin{i}, 'mode')
                        mode = varargin{i+1};
                    end    
                end
            end
            
            % Get number of elements
            Nb = length(obj.body);
            Nf = length(obj.force);
            
            % System variable definitions
            switch (obj.description.type)
                case 'fixed'
                    x = obj.dynamics.symbols.q(:);
                    dx = obj.dynamics.symbols.dq(:);
                    ddx = obj.dynamics.symbols.ddq(:);
                case 'floating'
                    x = obj.dynamics.symbols.q(:);
                    dx = obj.dynamics.symbols.u(:);
                    ddx = obj.dynamics.symbols.du(:);    
            end
                
            % Define the system inputs
            u_int = obj.dynamics.symbols.tau_a;
            u_ext = obj.dynamics.symbols.tau_e;
            
            % Define system arguments
            kinvars = {x, dx, ddx};
            dynvars = {x, dx, u_int, u_ext};
            
            proneu_info('Generating MATLAB functions for the model of the system: ');
            tstart = tic;
            
            
            if isparametrized == false
                % Check if special parameter values have been specified,
                % else use those stored internally.
                if isempty(paramvalues)
                    paramvalues = obj.parameters.values;
                end
                % Generate body-element functions
                for i=1:Nb
                    obj.body(i).generatefunctions('mode', mode, 'variables', kinvars, 'parameters', obj.parameters.symbols, 'values', paramvalues);
                end
                % Generate force-torque-element functions
                for j=1:Nf
                    obj.force(j).generatefunctions('mode', mode, 'variables', dynvars, 'parameters', obj.parameters.symbols, 'values', paramvalues);
                end
                % Generate numeric functions for the model dynamics 
                obj.dynamics.compute = obj.dynamics.symbols.generatefunctions('mode', mode, ...
                                                                              'variables', dynvars, ...
                                                                              'parameters', obj.parameters.symbols, ...
                                                                              'values', paramvalues);
                
            elseif isparametrized == true
                % Generate body-element functions
                for i=1:Nb
                    obj.body(i).generatefunctions('mode', mode, 'variables', kinvars, 'parameters', obj.parameters.symbols);
                end
                % Generate force-torque-element functions
                for j=1:Nf
                    obj.force(j).generatefunctions('mode', mode, 'variables', dynvars, 'parameters', obj.parameters.symbols);
                end
                % Generate numeric functions for the model dynamics 
                obj.dynamics.compute = obj.dynamics.symbols.generatefunctions('mode', mode, ...
                                                                              'variables', dynvars, ...
                                                                              'parameters', obj.parameters.symbols);
                
            end
            
            tend = toc(tstart);
            proneu_info(['---->Total computation time = ' num2str(tend) ' s']);
        
        end
        
    end
    
end

%
% HELPER FUNCTIONS
%

function [] = check_arguments(obj, q, tau_a, tau_e, I_a_g, arguments)
    
    % Define default configurations for the EoM generation
    name = 'robot';
    type = 'none';
    method = 'none';
    orientation = 'none';
    symsteps = 0;
    
    % Check if type and method have been defined by user
    if ~isempty(arguments)
        for i=1:numel(arguments)
            if ischar(arguments{i}) && strcmp(arguments{i}, 'name')
                name = arguments{i+1};
            elseif ischar(arguments{i}) && strcmp(arguments{i}, 'type')
                type = arguments{i+1};
            elseif ischar(arguments{i}) && strcmp(arguments{i}, 'method')
                method = arguments{i+1};
            elseif ischar(arguments{i}) && strcmp(arguments{i}, 'orientation')
                orientation = arguments{i+1};
            elseif ischar(arguments{i}) && strcmp(arguments{i}, 'symsteps')
                symsteps = arguments{i+1};
            end    
        end
    end
    
    % Check for argument errors
    if strcmp(type, 'fixed') && ~strcmp(orientation, 'none')
        error('Cannot define parametrization of base orientation if the system is fixed base.');
    end
    
    % Check if the base orientation paramatrization has been specified
    if strcmp(type, 'floating')
        if  strcmp(orientation, 'quaternion') || ...
            strcmp(orientation, 'angleaxis') || ...
            strcmp(orientation, 'taitbryanzyx') || ...
            strcmp(orientation, 'cardanxyz') || ...
            strcmp(orientation, 'eulerzyz') || ...
            strcmp(orientation, 'eulerzxz') || ...
            strcmp(orientation, 'custom')
            
            % Set the parametrization type
            obj.description.orientation = orientation;
       
        else
            
            % Else set to defaults
            warnmsg = {'A parametrization for base orientation was not ', ...
                       'specified. Default is to assume that it has been ', ...
                       'specified by the user.'};
            warnmsg = strcat(warnmsg{1}, warnmsg{2}, warnmsg{3});
            warning(warnmsg);
            obj.description.orientation = 'custom';
        end
    end
    
    % Set the type and method for the rigid body dynamics
    % formulation
    obj.name = name;
    obj.description.type = type;
    obj.description.method = method;
    obj.description.orientation = orientation;
    obj.description.symsteps = symsteps;
    
    % Store system arguments internally
    obj.modelargs.q = q;
    obj.modelargs.tau_a = tau_a;
    obj.modelargs.tau_e = tau_e;
    obj.modelargs.I_a_g = I_a_g;
    
end

function [] = define_multibody_system(obj, q_j, arguments)
    
    % Generate the vector of generalized velocities for the joints
    dq_j = genderivsym(q_j);

    % If the system is a floating-base, generate the 6-DoF
    % joint between inertial and local fixed base frames. This is
    % dependent on the parametrization used for orientation.
    if strcmp(obj.description.type, 'floating')
        
        % Generate the floating-base quantities
        [q, dq, u, F, G] = float_base(obj, q_j, dq_j, arguments);
        
        % Generate the floating-base dynamical properties
        obj.dynamics.symbols = FloatingBaseDynamics;
        
        % Set the system configuration and velocity quantities
        obj.dynamics.symbols.q = q;
        obj.dynamics.symbols.dq = dq;
        obj.dynamics.symbols.u = u;
        obj.dynamics.symbols.F = F;
        obj.dynamics.symbols.G = G;
        
        % Express dF,dG as functions of (q,u) and (q,dq) respectively
        obj.dynamics.symbols.dF = dAdt(F, q, F*u); 
        obj.dynamics.symbols.dG = dAdt(G, q, dq);
        
        % Generate derivative symbolic accelerations
        obj.dynamics.symbols.ddq = genderivsym(dq);
        obj.dynamics.symbols.du = genderivsym(u);
    else
        % Generate the fixed-base dynamical properties
        obj.dynamics.symbols = FixedBaseDynamics;
        
        % Set the system configuration and velocity quantities
        obj.dynamics.symbols.q = q_j;
        obj.dynamics.symbols.dq = dq_j;
        
        % Generate derivative symbolic acceleration
        obj.dynamics.symbols.ddq = genderivsym(dq_j);
    end
    
end

function [] = generate_model(obj)
    
    % Set model generator arguments
    switch (obj.description.type)
        case 'fixed'
            x = obj.dynamics.symbols.q;
            dx = obj.dynamics.symbols.dq;
            F = sym(eye(length(x)));
        case 'floating'
            x = obj.dynamics.symbols.q;
            dx = obj.dynamics.symbols.u;
            F = obj.dynamics.symbols.F;
    end
    
    % Define the system inputs
    I_a_g   = obj.modelargs.I_a_g;
    tau_a   = obj.modelargs.tau_a;
    tau_e   = obj.modelargs.tau_e;
    
    % Set the symbolic simplification level
    ns  = obj.description.symsteps;
    
    % Compute the dynamics of the system using an apropriate algorithm
    switch obj.description.method
        case 'proneu'
            proneu(obj, x, dx, F, tau_a, tau_e, I_a_g, ns);

        case 'eulerlagrange' 
            %
            % TODO
            %

        case 'featherstone' 
            %
            % TODO
            %
        otherwise
            error('Unsupported model generation method.');
    end
    
end

function  [q, dq, u, F, G] = float_base(obj, q_j, dq_j, arguments)
    
    % Generate the orientation as a rotation matrix if not
    % specified by the user
    if strcmp(obj.description.orientation, 'custom')
        
        % Initialzie custom base motion quantities
        F_IB = [];
        u_b = [];
        dq_b = [];
        
        % Retrieve the custom F matrix from user arguments
        for i=1:numel(arguments)
            if ischar(arguments{i}) && strcmp(arguments{i}, 'F_b')
                F = arguments{i+1};
            elseif ischar(arguments{i}) && strcmp(arguments{i}, 'u_b')
                u_b = arguments{i+1};
            elseif ischar(arguments{i}) && strcmp(arguments{i}, 'dq_b')
                dq_b = arguments{i+1};
            end   
        end
        
        if isempty(F_IB)
            error('Floating base parametrization is set to custom but "F_b" matrix has not been specified.');
        end
        
        if isempty(u_b)
            error('Floating base parametrization is set to custom but "u_b" vector has not been specified.');
        end
        
        if isempty(dq_b)
            error('Floating base parametrization is set to custom but "dq_b" vector has not been specified.');
        end
        
        % Ensure column vector form
        u_b = u_b(:);
        dq_b = dq_b(:);
        
        % Define the augmented generalized coordinates and velocities
        q = [dq_b; q_j];
        u = [u_b; dq_j];
        
    else
        % Generate floating base kinematics quantities
        [ I_r_IB, phi_IB, C_IB, dphi_IB, I_v_IB, B_omega_IB, F_I, F_B , G_I, G_B ] = genfltbody(obj.description.orientation, obj.body(1).name);
        
        % Set length of internal joints vector
        Nqj = length(q_j);
        
        % Define the configuration kinematics accordingly
        T_IB = sym(eye(4));
        T_IB(1:3,4) = I_r_IB;
        T_IB(1:3,1:3) = C_IB;
        
        % Define the generalized coordinates and velocities
        q = [I_r_IB; phi_IB; q_j];
        dq = [I_v_IB; dphi_IB; dq_j];
        
        % Also define the geometric generalized velocities (quasi-velocities)
        u = [I_v_IB; B_omega_IB; dq_j];
        
        % TODO: also provide the option for the following
        %u = [B_v_IB; B_omega_IB; dq_j]; -> use F_B, G_B
        %u = [I_v_IB; I_omega_IB; dq_j]; -> use F_I, G_I
        
        % Get the dimensions
        Nu = length(u);
        Nq = length(q);
        Nf = size(F_B);
        Ng = size(G_B);
        
        % Define the total F matrix
        F                       = sym(zeros(Nq,Nu));
        F(1:3,1:3)              = sym(eye(3));
        F(4:3+Nf(1),4:3+Nf(2))  = F_B;
        if Nqj > 0
            F(4+Nf(1):Nq,4+Nf(2):Nu)= eye(Nqj);
        end
        
        % Define the total G matrix
        G                       = sym(zeros(Nu,Nq));
        G(1:3,1:3)              = sym(eye(3));
        G(4:3+Ng(1),4:3+Ng(2))  = G_B;
        if Nqj > 0
            G(4+Ng(1):Nu,4+Ng(2):Nq)= eye(Nqj);
        end
        
        % Store the base configuration kinematics into the base body object
        obj.body(1).kinematics.symbols.T_PiBi = T_IB;
    end
end

function [] = collect_model_parameters(obj)

    % Set system arguments and definitinos
     switch (obj.description.type)
        case 'fixed'
            x = obj.dynamics.symbols.q;
            dx = obj.dynamics.symbols.dq;
        case 'floating'
            x = obj.dynamics.symbols.q;
            dx = obj.dynamics.symbols.u;
    end
    
    % Set the symbolic simplification level
    tau_a = obj.modelargs.tau_a;
    tau_e = obj.modelargs.tau_e;
    I_a_g = obj.modelargs.I_a_g;

    % Initialize parameters array
    params = [];
    
    % Collect body parameters
    for i=1:numel(obj.body)
        bparam = obj.body(i).getparameters();
        bparam = bparam.model(:);
        params = [params; bparam];
    end
    
    % Collect force element parameters
    for j=1:numel(obj.force)
        
        % Get raw parameter set from each element
        ftparam = obj.force(j).getparameters();
        ftparam = ftparam.model(:);
        
        % Remove the symbols defined as external inputs
        ftparam = subs(ftparam, tau_a, zeros(size(tau_a(:),1),1));
        ftparam = subs(ftparam, tau_e, zeros(size(tau_e(:),1),1));
        ftparam = symvar(ftparam);
        ftparam = ftparam(:);
        
        % Append to parameters array
        params = [params; ftparam];
    end
    
    % Append external parameters
    params = [params; I_a_g(:)];
    
    % Extract unique parameters list array
    params = symvar(params);
    params = params(:);
    
    % Remove joint force-torque variables and state variables
    params = subs(params, x(:), sym(zeros(numel(x),1)));
    params = subs(params, dx(:), sym(zeros(numel(dx),1)));
    params = symvar(params);
    params = params(:);
    
    % Store the unique parameters array
    obj.parameters.symbols = params;
end
