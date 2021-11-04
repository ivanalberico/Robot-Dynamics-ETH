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

function [ body ] = bodykinext(x, dx, ddx, F, base, parent, body, ns)

    %%% Compute Position & Orientation Kinematics

    % Body configurations
    if isempty(base) && isempty(parent)
        body.T_BBi = sym(eye(4));
        C_IB = body.T_IBi(1:3,1:3);
    else
        body.T_BBi = simplify(parent.T_BBi * body.T_PiBi, ns);
        C_IB = base.T_IBi(1:3,1:3);
    end
    
    % Base-to-Body CoM configurations
    body.T_BCi = simplify(body.T_BBi * body.T_BiCi, ns);
    
    % World-to-Body CoM configurations
    body.T_ICi = simplify(body.T_IBi * body.T_BiCi, ns);
    
    % Set the body geometric quantities of the body
    C_BBi = body.T_BBi(1:3,1:3);
    B_r_BBi = body.T_BBi(1:3,4);
    B_r_BCi = body.T_BCi(1:3,4);
    I_r_ICi = body.T_ICi(1:3,4);
    
    %%% Compute Velocity Kinematics
    
    % Angular velocities
    body.B_omega_BBi = simplify(unskew(dAdt(C_BBi, x, F*dx) * C_BBi.'), ns);
    body.I_omega_BBi = simplify( C_IB * body.B_omega_BBi, ns);
    body.B_omega_IBi = simplify(C_IB.' * body.I_omega_IBi, ns);
    body.I_omega_BCi = body.I_omega_BBi;
    body.B_omega_BCi = body.B_omega_BBi;
    body.B_omega_ICi = body.B_omega_IBi;
    
    % Linear velocities
    body.B_v_BBi = simplify(dAdt(B_r_BBi, x, F*dx), ns);
    body.I_v_BBi = simplify(C_IB * body.B_v_BBi, ns);
    body.B_v_IBi = simplify(C_IB.' * body.I_v_IBi, ns);
    body.B_v_BCi = simplify(dAdt(B_r_BCi, x, F*dx), ns);
    body.I_v_BCi = simplify(C_IB * body.B_v_BCi, ns);
    body.I_v_ICi = simplify(dAdt(I_r_ICi, x, F*dx), ns);
    body.B_v_ICi = simplify(C_IB.' * body.I_v_ICi, ns);
    
    % Intrinisc geometric Jacobians matrices
    body.B_J_BBi = simplify( jacobian( [body.B_v_BBi; body.B_omega_BBi], dx), ns );
    body.I_J_BBi = simplify( jacobian( [body.I_v_BBi; body.I_omega_BBi], dx), ns );
    body.B_J_BCi = simplify( jacobian( [body.B_v_BCi; body.B_omega_BCi], dx), ns );
    body.I_J_BCi = simplify( jacobian( [body.I_v_BCi; body.I_omega_BCi], dx), ns );

    % Extrinsic geometric Jacobians matrices
    body.B_J_IBi = simplify([C_IB.' zeros(3,3); zeros(3,3) C_IB.'] * body.I_J_IBi, ns );
    body.B_J_ICi = simplify([C_IB.' zeros(3,3); zeros(3,3) C_IB.'] * body.I_J_ICi, ns );

    %%% Compute Acceleration Kinematics
    
    % Derivative of the velocity rate map matrix
    dF = simplify(dAdt(F, x, F*dx), ns);
    
    % Compute the derivatives of the Jacobian matrices
    body.B_dJ_BBi = simplify( dAdt(body.B_J_BBi, x, F*dx), ns);
    body.I_dJ_BBi = simplify( dAdt(body.I_J_BBi, x, F*dx), ns);
    body.I_dJ_IBi = simplify( dAdt(body.B_J_IBi, x, F*dx), ns);
    body.B_dJ_IBi = simplify( dAdt(body.B_J_IBi, x, F*dx), ns);
    body.B_dJ_BCi = simplify( dAdt(body.B_J_BCi, x, F*dx), ns);
    body.I_dJ_BCi = simplify( dAdt(body.I_J_BCi, x, F*dx), ns);
    body.B_dJ_ICi = simplify( dAdt(body.B_J_ICi, x, F*dx), ns);
    
    % Compute spatial accelerations
    B_dw_BBi = body.B_dJ_BBi*F*dx + body.B_J_BBi*(F*ddx + dF*dx);
    I_dw_BBi = body.I_dJ_BBi*F*dx + body.I_J_BBi*(F*ddx + dF*dx);
    I_dw_IBi = body.I_dJ_IBi*F*dx + body.I_J_IBi*(F*ddx + dF*dx);
    B_dw_IBi = body.B_dJ_IBi*F*dx + body.B_J_IBi*(F*ddx + dF*dx);
    B_dw_BCi = body.B_dJ_BCi*F*dx + body.B_J_BCi*(F*ddx + dF*dx);
    I_dw_BCi = body.I_dJ_BCi*F*dx + body.I_J_BCi*(F*ddx + dF*dx);
    I_dw_ICi = body.I_dJ_ICi*F*dx + body.I_J_ICi*(F*ddx + dF*dx);
    B_dw_ICi = body.B_dJ_ICi*F*dx + body.B_J_ICi*(F*ddx + dF*dx);
    
    % Set the cartesian accelerations
    body.B_a_BBi    = simplify( B_dw_BBi(1:3), ns);
    body.B_psi_BBi  = simplify( B_dw_BBi(4:6), ns);
    body.I_a_BBi    = simplify( I_dw_BBi(1:3), ns);
    body.I_psi_BBi  = simplify( I_dw_BBi(4:6), ns);
    body.I_a_IBi    = simplify( I_dw_IBi(1:3), ns);
    body.I_psi_IBi  = simplify( I_dw_IBi(4:6), ns);
    body.B_a_IBi    = simplify( B_dw_IBi(1:3), ns);
    body.B_psi_IBi  = simplify( B_dw_IBi(4:6), ns);
    body.B_a_BCi    = simplify( B_dw_BCi(1:3), ns);
    body.B_psi_BCi  = simplify( B_dw_BCi(4:6), ns);
    body.I_a_BCi    = simplify( I_dw_BCi(1:3), ns);
    body.I_psi_BCi  = simplify( I_dw_BCi(4:6), ns);
    body.I_a_ICi    = simplify( I_dw_ICi(1:3), ns);
    body.I_psi_ICi  = simplify( I_dw_ICi(4:6), ns);
    body.B_a_ICi    = simplify( B_dw_ICi(1:3), ns);
    body.B_psi_ICi  = simplify( B_dw_ICi(4:6), ns);
    
end
