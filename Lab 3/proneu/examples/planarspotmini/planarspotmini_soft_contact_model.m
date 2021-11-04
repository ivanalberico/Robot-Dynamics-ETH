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

%%
%   File:           planarspotmini_soft_contact_model.m
%
%   Authors:        Vassilios Tsounis, tsounisv@ethz.ch
%
%   Date:           3/11/2017
%
%   Desciption:     A ground-force contact model using soft contacts and a
%                   point foot assumption.
%

function [ wc ] = planarspotmini_soft_contact_model(model,logger,t,x)
    
    
    % Set simulation data
    mparams = model.parameters.values;
    
    % Get the system generalized coordinates and velocities
    q_data = x(1:10);
    dq_data = x(11:20);
    
    % Extract kinematic information of the rear foot.
    T_Rc = model.body(4).kinematics.compute.T_IBi(q_data,[],[],mparams);
    J_Rc = model.body(4).kinematics.compute.I_J_IBi(q_data,[],[],mparams);

    % Extract kinematic information of the front foot.
    T_Fc = model.body(7).kinematics.compute.T_IBi(q_data,[],[],mparams);
    J_Fc = model.body(7).kinematics.compute.I_J_IBi(q_data,[],[],mparams);
    
    % Extract kinematic information of the arm's harnd.
    T_Ac = model.body(11).kinematics.compute.T_IBi(q_data,[],[],mparams);
    J_Ac = model.body(11).kinematics.compute.I_J_IBi(q_data,[],[],mparams);
    
    %
    wcr = compute_contact_force(t, T_Rc, J_Rc, dq_data, 1);
    wcf = compute_contact_force(t, T_Fc, J_Fc, dq_data, 2);
%     fca = compute_contact_force(t, T_Ac, J_Ac, dq_data, 3);
    wca = zeros (6,1);
    
    %
    wc = [wcr; wcf; wca];
end


%
% HELPER FUNCTIONS
%

function [ wc ] = compute_contact_force(t, Tc, J_c, dq, index)

    %
    % Compliant contact model
    %
    
    % Contact model parameters
    z_Floor = 0;
    Tol_gamma = 1.0e-6;
    mu = 0.9;
    c_N = 1.0e+4;
    d_N = 1.0e+2; 
    c_T = 1.0e+4;
    d_T = 1.0e+2;    
    
    % Parameters
    persistent first_contact_detected;
    persistent is_slipping;
    persistent lambda_N;
    persistent lambda_T;
    persistent P_0;
    persistent P_k;
    persistent Q_k;
    persistent Q_0;
    
    % Initialize persistent variables
    if (t == 0)
       first_contact_detected = [true true true];
       is_slipping = [false false false];
       lambda_N = [0 0 0];
       lambda_T = zeros(2,3);
       P_0 = [0 0 0];
       P_k = [0 0 0];
       Q_k = zeros(2,3);
       Q_0 = zeros(2,3);
    end
    
    % Get foot coordinates
    x_Fk = Tc(1,4);
    y_Fk = Tc(2,4);
    z_Fk = Tc(3,4);
    
    % Normal gap function for contact detection
    g_N_check = z_Fk - z_Floor;
    
    % Check for closed contacts
    if g_N_check <= 0
        
        % Check for first contact point
        if first_contact_detected(index) == true
            P_0(:,index) = z_Fk;
            Q_0(:,index) = [x_Fk y_Fk].';
            first_contact_detected(index) = false;
        end
        
        % Update contact compliance variables
        P_k(:,index) = z_Fk;
        Q_k(:,index) = [x_Fk y_Fk].';
        
        % Compute gap functions
        g_N = P_k(:,index) - P_0(:,index);
        g_T = Q_k(:,index) - Q_0(:,index);
        
        % Relative gap velocity
        gamma_c = J_c*dq;
        gamma_N = gamma_c(3);
        gamma_T = gamma_c(1:2);
        
        % Compliant contact algorithm, explained in [1]
        lambda_N(index) = max(-c_N*g_N - d_N*gamma_N,0);
        if is_slipping(index) == true
            if (norm(gamma_T,2) <= Tol_gamma) || (norm(lambda_T(:,index),2) <= mu*lambda_N(index))
                is_slipping(index) = false;
                Q_0(:,index) = [x_Fk y_Fk].';
            end
        else
            lambda_T(:,index) = -c_T*g_T - d_T*gamma_T;
            if norm(lambda_T(:,index),2) >= mu*lambda_N(index)
                lambda_T(:,index) = -mu*lambda_N(index)*(gamma_T/norm(gamma_T,2));
                is_slipping(index) = true;
            end
        end
    else
        first_contact_detected(index) = true;
        % Forces are zero
        lambda_N(index) = 0;
        lambda_T(:,index) = [0 0].';
    end
    
    % Set results
    f_x = lambda_T(1,index);
    f_y = lambda_T(2,index);
    f_z = lambda_N(index);
    
    
    % Set total external wrench
    wc = [f_x f_y f_z zeros(1,3)].';

end


%% References
%
% [1]   C. Gehring, R. Diethelm, R. Siegwart, G. Nuetzi, R. Leine. "An Evaluation
%       of Moreau's Time-Stepping Scheme for the Simulation of a Legged Robot." 2014
% 

%%
% EOF
