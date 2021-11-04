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

function [ body ] = bodykinmin(q, u, F, parent, body, ns)

    %%% Compute Position & Orientation Kinematics
    
    % If parent is empty then the body is the base
    if isempty(parent)
        % Set the root node configuration
        body.T_IBi = body.T_PiBi;
    else
        % Compute recursion on parent's kinematics
        body.T_IBi = simplify(parent.T_IBi * body.T_PiBi, ns);
    end
    
    % Set the body absolute body configuration
    C_IBi = body.T_IBi(1:3,1:3);
    I_r_IBi = body.T_IBi(1:3,4);
    
    %%% Compute Velocity Kinematics
    
    % Compute body veclitites
    body.I_v_IBi = simplify(dAdt(I_r_IBi, q, F*u), ns); 
    body.I_omega_IBi = simplify(unskew(dAdt(C_IBi, q, F*u) * C_IBi.'), ns);
    body.I_omega_ICi = body.I_omega_IBi;
    
    % Compute body Jacobian
    body.I_J_IBi = simplify(jacobian([body.I_v_IBi; body.I_omega_IBi], u), ns);
    
    % Compute CoM body Jacobian
    body.I_J_ICi = sym(zeros(6,length(u)));
    Bi_r_BiCi = body.T_BiCi(1:3,4);
    I_r_BiCi = simplify(C_IBi * Bi_r_BiCi, ns);
    body.I_J_ICi(1:3,:) = body.I_J_IBi(1:3,:) - skew(I_r_BiCi) * body.I_J_IBi(4:6,:);
    body.I_J_ICi(4:6,:) = body.I_J_IBi(4:6,:);
    body.I_J_ICi = simplify(body.I_J_ICi, ns);
    
    %%% Compute Acceleration Kinematics
    
    % Compute CoM body Jacobian derivative
    body.I_dJ_IBi = simplify(dAdt(body.I_J_IBi, q, F*u), ns);
    body.I_dJ_ICi = simplify(dAdt(body.I_J_ICi, q, F*u), ns);
    
end
