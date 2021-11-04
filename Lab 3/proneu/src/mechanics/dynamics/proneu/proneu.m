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
% function [] = proneu(model, q, u, F, tau_a, tau_e, I_a_g, ns)
% 
% TODO
%
%

function [] = proneu(model, q, u, F, tau_a, tau_e, I_a_g, ns)
    
    % Check arguments
    if ~isa(model, 'RobotModel')
        error('First argument is not a RobotModel type');
    end
    
    % Get body and force element dimensions
    Nrb = length(model.body);
    Nft = length(model.force);
    
    % Set system dimensions
    Nu = length(u);
    %Nta = length(tau_a);
    %Nte = length(tau_e);
    
    % Initialize computation counters
    tkinbodycalc = zeros(Nrb,1);
    tdynbodycalc = zeros(Nrb,1);
    tforcecalc = zeros(Nft,1);
    tdynsimplify = 0;
    
    %%%
    % Set system definitions
    %%%

    % Ensure that all vectors are column-vectors
    q = q(:);
    u = u(:);
    tau_a = tau_a(:);
    tau_e = tau_e(:);
    I_a_g = I_a_g(:);
    
    %%%
    % Kinematics of the spanning tree
    %%%
    
    proneu_info('Computing system kinematics: ');
    
    %%% Compute the kinematics of the base (single child of root node)
    
    proneu_info(['-->Computing kinematics of the base at body ' model.body(1).name '...']);
    tstart = tic;
    if (Nu >= 6)
        model.body(1).kinematics.symbols = bodykinminalt(q, u, F, [], model.body(1).kinematics.symbols, ns);
    else
        model.body(1).kinematics.symbols = bodykinmin(q, u, F, [], model.body(1).kinematics.symbols, ns);
    end
    tkinbodycalc(1) = toc(tstart);
    
    %%% Compute the body kinematics of the child nodes
    
    % Propagate kinematics properties forward/down the tree
    for i=2:Nrb
        
        % Determine the parent of the current body
        p = model.body(i).ktree.parents(1);
        
        % Display console notification
        proneu_info(['-->Computing kinematics of body {' num2str(i) ', ' model.body(i).name ...
                     '}, with parent {' num2str(p) ', ' model.body(p).name '}.']);
        
        % Compute the body kinematics
        tstart = tic;
        if (Nu >= 6)
            model.body(i).kinematics.symbols = bodykinminalt(q, u, F, model.body(p).kinematics.symbols, model.body(i).kinematics.symbols, ns);
        else
            model.body(i).kinematics.symbols = bodykinmin(q, u, F, model.body(p).kinematics.symbols, model.body(i).kinematics.symbols, ns);
        end
        tkinbodycalc(i) = toc(tstart);
    end
    
    %%%
    % Unactuated Dynamics of the system of bodies
    %%%
    
    proneu_info('Computing intrinsic dynamics of the multi-body system: ');
    
    % Initialize iterated quantities 
    model.dynamics.symbols.M    = sym(zeros(Nu,Nu));
    model.dynamics.symbols.b    = sym(zeros(Nu,1));
    model.dynamics.symbols.g    = sym(zeros(Nu,1));
    model.dynamics.symbols.tau  = sym(zeros(Nu,1));
    
    % Compute the intrinsic body dynamics - simplifications are disabled
    for i=1:Nrb
        % Display console notification
        proneu_info(['-->Computing dynamics of body {' num2str(i) ', ' model.body(i).name '}.'] );
        tstart = tic;
        
        % Compile the generalized mass-matrix
        Mi = bodymassmatrix(model.body(i).inertia.parameters.m_Bi, ...
                            model.body(i).kinematics.symbols.T_BiCi(1:3,4), ...
                            model.body(i).inertia.parameters.Bi_Theta_Bi, ...
                            model.body(i).kinematics.symbols.T_IBi(1:3,1:3), ...
                            model.body(i).kinematics.symbols.I_J_ICi, ...
                            ns);
        
        % Compile the generalized non-linear forces
        bi = bodynonlinearforces(u, ...
                                 model.body(i).inertia.parameters.m_Bi, ...
                                 model.body(i).kinematics.symbols.T_BiCi(1:3,4), ...
                                 model.body(i).inertia.parameters.Bi_Theta_Bi, ...
                                 model.body(i).kinematics.symbols.T_IBi(1:3,1:3), ...
                                 model.body(i).kinematics.symbols.I_omega_ICi, ...
                                 model.body(i).kinematics.symbols.I_J_ICi, ...
                                 model.body(i).kinematics.symbols.I_dJ_ICi, ...
                                 ns);
        
        % Compile the generalized gravity forces
        gi = bodygravityforce(model.body(i).inertia.parameters.m_Bi, ...
                              model.body(i).kinematics.symbols.I_J_ICi, ...
                              I_a_g, ...
                              ns);
             
        % Collect all terms over for all bodies
        model.dynamics.symbols.M = model.dynamics.symbols.M + Mi;
        model.dynamics.symbols.b = model.dynamics.symbols.b + bi;
        model.dynamics.symbols.g = model.dynamics.symbols.g + gi;
        
        % Get dynamics computation time
        tdynbodycalc(i) = toc(tstart);
    end
    
    % Remove unused memory
    clear Mi bi gi;
    
    %%%
    % Generalized forces from external sources and actuators
    %%%
    
    proneu_info('Computing extrinsic dynamics of the applied forces: ');
    
    % Compute the external force contributions
    for j=1:Nft
        
        % Display console notification
        proneu_info(['-->Computing generalized forces of force-torque element {' num2str(j) ', ' model.force(j).name '}.'] );
        tic;

        % Set the body-pair between which the force/torque element acts
        p = model.force(j).ktree.parent;
        c = model.force(j).ktree.child;
        
        % Determine if the parent body is the "world".
        if p == 0
            % Compute the generalized force contribution
            model.force(j).symbols = bodyexternalforce([], model.body(c).kinematics.symbols, model.force(j).symbols, ns);
        else
            % Compute the generalized force contribution
            model.force(j).symbols = bodyexternalforce(model.body(p).kinematics.symbols, model.body(c).kinematics.symbols, model.force(j).symbols, ns);
        end
        
        % Compile the generalized external force vector
        model.dynamics.symbols.tau = model.dynamics.symbols.tau + model.force(j).symbols.tau_j;
        model.dynamics.symbols.tau = simplesym(model.dynamics.symbols.tau, 'elementwise', ns);
        
        % Get dynamics computation time
        tforcecalc(j) = toc;
    end
    
    % Compute the actuation selection matrix
    model.dynamics.symbols.S_a = jacobian(model.dynamics.symbols.tau, tau_a(:)).';
    
    % Set the actuator generalized force vector
    model.dynamics.symbols.tau_a = tau_a(:);
    
    % Compute the actuation selection matrix
    model.dynamics.symbols.S_e = jacobian(model.dynamics.symbols.tau, tau_e(:)).';
    
    % Set the actuator generalized force vector
    model.dynamics.symbols.tau_e = tau_e(:);
    
    % Compute the total external generalized forces due to actuation
    model.dynamics.symbols.tau_act = model.dynamics.symbols.S_a.' * model.dynamics.symbols.tau_a;
    
    % Compute the total external generalized forces due to the environment
    model.dynamics.symbols.tau_ext = model.dynamics.symbols.S_e.' * model.dynamics.symbols.tau_e;
    
    %%%
    % Explicit enforcement of the unit quaternion contstraint
    %%%
    
    % If the base orientation is parametrized using quaternions enforce
    % unit quaternion condition
    if strcmp(model.description.type,'floating') && strcmp(model.description.orientation,'quaternion')
        unit_quaternion_simplification(model);
    end
    
    %%%
    % Performance report
    %%%
    
    proneu_info('Performance Report: ');
    
    proneu_info('-->Kinematics: ');
    
    for i=1:Nrb
        % Display console notification
        proneu_info(['---->Body {' num2str(i) ', ' model.body(i).name '}: ' num2str(tkinbodycalc(i)) ' s'] );
    end
    
    proneu_info('-->Unactuated Dynamics: ');
    for i=1:Nrb
        % Display console notification
        proneu_info(['---->Body {' num2str(i) ', ' model.body(i).name '}: ' num2str(tdynbodycalc(i)) ' s'] );
    end
    
    proneu_info(['---->Simplifications: ' num2str(tdynsimplify) ' s']);
    
    proneu_info('-->External Force-Torque Dynamics: ');
    
    for j=1:Nft
        % Display console notification
        proneu_info(['---->Force {' num2str(j) ', ' model.force(j).name '}: ' num2str(tforcecalc(j)) ' s'] );
    end
    
end


%
% HELPER FUNCTIONS
%

function [] = unit_quaternion_simplification(model)
    
    %
    q = model.dynamics.symbols.q;

    %
    p_0 = q(4);
    p_1 = q(5);
    p_2 = q(6);
    p_3 = q(7);
    
    %
    expr = p_0^2+p_1^2+p_2^2+p_3^2;
    
    %
    for i=1:length(model.body)
        model.body(i).kinematics.symbols.T_IBi = subs(model.body(i).kinematics.symbols.T_IBi, expr, 1);
        model.body(i).kinematics.symbols.I_v_IBi = subs(model.body(i).kinematics.symbols.I_v_IBi, expr, 1);
        model.body(i).kinematics.symbols.I_omega_IBi = subs(model.body(i).kinematics.symbols.I_omega_IBi, expr, 1);
        model.body(i).kinematics.symbols.I_J_IBi = subs(model.body(i).kinematics.symbols.I_J_IBi, expr, 1);
        model.body(i).kinematics.symbols.I_omega_ICi = subs(model.body(i).kinematics.symbols.I_omega_ICi, expr, 1);
        model.body(i).kinematics.symbols.I_J_ICi = subs(model.body(i).kinematics.symbols.I_J_ICi, expr, 1);
        model.body(i).kinematics.symbols.I_dJ_ICi = subs(model.body(i).kinematics.symbols.I_dJ_ICi, expr, 1);
    end
    
    %
    for j=1:length(model.force)
        model.force(j).symbols.I_F_j = subs(model.force(j).symbols.I_F_j, expr, 1);
        model.force(j).symbols.I_T_j = subs(model.force(j).symbols.I_T_j, expr, 1);
        model.force(j).symbols.I_J_j = subs(model.force(j).symbols.I_J_j, expr, 1);
        model.force(j).symbols.tau_j = subs(model.force(j).symbols.tau_j, expr, 1); 
    end
    
    %
    model.dynamics.symbols.M = subs(model.dynamics.symbols.M, expr, 1);
    model.dynamics.symbols.b = subs(model.dynamics.symbols.b, expr, 1);
    model.dynamics.symbols.g = subs(model.dynamics.symbols.g, expr, 1);
    model.dynamics.symbols.tau = subs(model.dynamics.symbols.tau, expr, 1);
    model.dynamics.symbols.tau_act = subs(model.dynamics.symbols.tau_act, expr, 1);
    model.dynamics.symbols.tau_ext = subs(model.dynamics.symbols.tau_ext, expr, 1);
    model.dynamics.symbols.S_a = subs(model.dynamics.symbols.S_a, expr, 1);
    model.dynamics.symbols.S_e = subs(model.dynamics.symbols.S_e, expr, 1);
end
