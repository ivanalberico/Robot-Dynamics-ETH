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

function [ body ] = bodykinminalt(q, u, F, parent, body, ns)
    
    % If parent is empty then the body is the base
    if isempty(parent)
        % Set the root node configuration
        body.T_IBi = body.T_PiBi;
        
        % Get parent configuration kinematics
        C_IPi = sym(eye(3));
        I_J_IPi = sym(zeros(6,length(u)));
    else
        % Compute recursion on parent's kinematics
        body.T_IBi = simplify(parent.T_IBi * body.T_PiBi, ns);
        
        % Get parent configuration kinematics
        C_IPi = parent.T_IBi(1:3,1:3);
        I_J_IPi = parent.I_J_IBi;
    end
    
    % Set the body absolute configuration kinematics
    C_PiBi = body.T_PiBi(1:3,1:3);
    C_IBi = body.T_IBi(1:3,1:3);
    I_r_BiCi = simplify(C_IBi * body.T_BiCi(1:3,4), ns);
    
    % Set the parent-to-body kinematics
    I_r_PiBi = simplify(C_IPi * body.T_PiBi(1:3,4), ns);
    I_v_PiBi = simplify(dAdt(I_r_PiBi, q, F*u), ns);
    I_omega_PiBi = simplify(unskew(C_IPi * dAdt(C_PiBi, q, F*u) * C_IBi.'), ns); 
    I_J_PiBi = simplify(jacobian([I_v_PiBi; I_omega_PiBi], u), ns);
    
    % Compute the body jacobian
    body.I_J_IBi = sym(zeros(6,length(u)));
    body.I_J_IBi = I_J_IPi + I_J_PiBi;
    body.I_J_IBi = simplesym(body.I_J_IBi ,'elementwise', ns);
    
    % Compute the CoM jacobian
    body.I_J_ICi = sym(zeros(6,length(u)));
    body.I_J_ICi(1:3,:) = body.I_J_IBi(1:3,:) - skew(I_r_BiCi) * body.I_J_IBi(4:6,:);
    body.I_J_ICi(4:6,:) = body.I_J_IBi(4:6,:);
    body.I_J_ICi = simplesym(body.I_J_ICi ,'elementwise', ns);
    
    % Compute body velocities
    body.I_v_IBi = simplify(body.I_J_IBi(1:3,:)*u, ns);
    body.I_v_ICi = simplify(body.I_J_ICi(1:3,:)*u, ns);
    body.I_omega_IBi = simplify(body.I_J_IBi(4:6,:)*u, ns);
    body.I_omega_ICi = simplify(body.I_J_ICi(4:6,:)*u, ns);
   
    % Compute CoM body Jacobian derivative
    body.I_dJ_IBi = simplify(dAdt(body.I_J_IBi, q, F*u), ns);
    body.I_dJ_ICi = simplify(dAdt(body.I_J_ICi, q, F*u), ns);
    
end
