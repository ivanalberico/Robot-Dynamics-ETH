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

function [ force ] = bodyexternalforce(parent, body, force, ns)

    %%% Action force acting on the body
    
    % Get the action force point of application
    Bi_r_BiQi = force.Bi_r_BiAj;
    
    % Get the body orientation
    C_IBi = body.T_IBi(1:3,1:3);
    
    % Get the body jacobians
    I_J_P_IBi = body.I_J_IBi(1:3,:);
    I_J_R_IBi = body.I_J_IBi(4:6,:);
    
    % Compute Jacobian at the action force application point 
    I_J_P_IQi = I_J_P_IBi - skew(C_IBi * Bi_r_BiQi) * I_J_R_IBi;
    
    % Compute total action Jacobian
    I_J_Aj = [I_J_P_IQi; I_J_R_IBi];
    
    % Check whether the forces/torque element has been defined using an 
    % inertial or local body frame 
    
    if isempty(force.Bi_F_j) && ~isempty(force.I_F_j)
        I_F_j = force.I_F_j;
        force.Bi_F_j = simplify(C_IBi.' * I_F_j, ns);
        
    elseif ~isempty(force.Bi_F_j) && isempty(force.I_F_j)
        I_F_j = simplify(C_IBi * force.Bi_F_j, ns);
        force.I_F_j = I_F_j;
        
    end
    
    if isempty(force.Bi_T_j) && ~isempty(force.I_T_j)
        I_T_j = force.I_T_j;
        force.Bi_T_j = simplify(C_IBi.' * I_T_j, ns);
        
    elseif ~isempty(force.Bi_T_j) && isempty(force.I_T_j)
        I_T_j = simplify(C_IBi * force.Bi_T_j, ns);
        force.I_T_j = I_T_j;
        
    end
    
    % Compute the action force acting on the body
    tau_A = I_J_Aj.' * [I_F_j; I_T_j];
    
    %%% Reaction force acting on the parent
    
    % If parent is empty then there is no reaction force
    if isempty(parent)
        % Get dimensions of the action Jacobian matrix
        Ndx = size(I_J_Aj, 2);
        
        % Set the reaction Jacobian to a zeros matrix
        I_J_Rj = sym(zeros(6, Ndx));
        
        % Zero reaction force if it is purely external
        tau_R = sym(zeros(Ndx,1));
    else
        
        % Get the reaction force point of application
        Bk_r_BkQk = force.Pi_r_PiRj;
        
        % Get the body orientation
        C_IBk = parent.T_IBi(1:3,1:3);
        
        % Get the parent jacobians
        I_J_P_IBk = parent.I_J_IBi(1:3,:);
        I_J_R_IBk = parent.I_J_IBi(4:6,:);
        
        % Compute Jacobian at the action force application point 
        I_J_P_IQk = I_J_P_IBk - skew(C_IBk * Bk_r_BkQk) * I_J_R_IBk; 
        
        % Compute total action Jacobian
        I_J_Rj = [I_J_P_IQk; I_J_R_IBk];
        
        % Compute the reaction force acting on the parent
        tau_R = I_J_Rj.' * [-I_F_j;- I_T_j];
    end

    %%% Total generalized force
    
    % Compute total generalized force contribution
    force.tau_j = tau_A + tau_R;
    
    % Store the force/torque element Jacobian
    force.I_J_j = I_J_Aj + I_J_Rj;
    
    % Perform element-wise implifications
    force.tau_j = simplesym(force.tau_j, 'elementwise', ns);
    force.I_J_j = simplesym(force.I_J_j, 'elementwise', ns);
    
end
