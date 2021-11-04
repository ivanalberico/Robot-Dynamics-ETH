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
%   File:           monopod_soft_contact_model.m
%
%   Authors:        Vassilios Tsounis, tsounisv@ethz.ch
%
%   Date:           1/2/2017
%
%   Desciption:     A ground-force contact model using soft contacts and a
%                   point foot assumption.
%

function [ W_F ] = monopod_soft_contact_model(model,logger,t,x)
    
    % Set simulation data
    qdata = x(1:3);
    dqdata = x(4:6);
    mparams = model.parameters.values;
    
    % Extract kinematic information of the foot.
    T_c = model.body(4).kinematics.compute.T_IBi(qdata,[],[],mparams);
    J_c = model.body(4).kinematics.compute.I_J_IBi(qdata,[],[],mparams);

    %
    % Compliant contact model
    %
    
    % Parameters
    persistent first_contact_detected;
    persistent is_slipping;
    persistent lambda_N;
    persistent lambda_T;
    persistent P_0;
    persistent P_k;
    persistent Q_k;
    persistent Q_0;
    
    % Constants
    z_Floor = 0;
    Tol_gamma = 1.0e-6;
    mu = 0.4;
    c_N = 1.0e+5;
    d_N = 1.0e+4; 
    c_T = 1.0e+5;
    d_T = 1.0e+4;    
    
    % Initialize persistent variables
    if (t == 0)
       first_contact_detected = true;
       is_slipping = false;
       lambda_N = 0;
       lambda_T = [0 0].';
       P_0 = 0;
       P_k = 0;
       Q_k = [0 0].';
       Q_0 = [0 0].';
    end
    
    % Get foot coordinates
    x_Fk = T_c(1,4);
    y_Fk = T_c(2,4);
    z_Fk = T_c(3,4);
    
    % Normal gap function for contact detection
    g_N_check = z_Fk - z_Floor;
    
    % Check for closed contacts
    if g_N_check <= 0
        
        % Check for first contact point
        if first_contact_detected == true
            P_0 = z_Fk;
            Q_0 = [x_Fk y_Fk].';
            first_contact_detected = false;
        end
        
        % Update contact compliance variables
        P_k = z_Fk;
        Q_k = [x_Fk y_Fk].';
        
        % Compute gap functions
        g_N = P_k - P_0;
        g_T = Q_k - Q_0;
        
        % Relative gap velocity
        gamma_c = J_c*dqdata;
        gamma_N = gamma_c(3);
        gamma_T = gamma_c(1:2);
        
        % Compliant contact algorithm, explained in [1]
        lambda_N = max(-c_N*g_N - d_N*gamma_N,0);
        if is_slipping == true;
            if (norm(gamma_T,2) <= Tol_gamma) || (norm(lambda_T,2) <= mu*lambda_N)
                is_slipping = false;
                Q_0 = [x_Fk y_Fk].';
            end
        else
            lambda_T = -c_T*g_T - d_T*gamma_T;
            if norm(lambda_T,2) >= mu*lambda_N
                lambda_T = -mu*lambda_N*(gamma_T/abs(gamma_T));
                is_slipping = true;
            end
        end
    else
        first_contact_detected = true;
        % Forces are zero
        lambda_N = 0;
        lambda_T = [0 0].';
    end
    
    % Set results
    F_Fx = lambda_T(1);
    F_Fy = lambda_T(2);
    F_Fz = lambda_N;
    T_Fx = 0.0;
    T_Fy = 0.0;
    T_Fz = 0.0;
    
    % Set total external wrench
    W_F = [F_Fx F_Fy F_Fz T_Fx T_Fy T_Fz].';
    
end

%% References
%
% [1]   C. Gehring, R. Diethelm, R. Siegwart, G. N�utzi, R. Leine. "An Evaluation
%       of Moreau�s Time-Stepping Scheme for the Simulation of a Legged Robot." 2014
% 

%%
% EOF
