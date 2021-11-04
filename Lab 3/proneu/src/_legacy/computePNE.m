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

% function [sys, body, ftel] = computePNE(body,ftel,q,dq,tau,I_a_grav)
% 
% INPUT:
% * body:       kinematic tree
% * ftel:       force/torque elements
% * q:          generalized coordinates (symb) in desired order
% * Dq:         coresponding velocities
% * tau:        actuator force/torque array (symb)
% * I_a_grav:   gravity vector (R3)
%
% OUTPUT:
% * sys:        system struct containing dynamcis (all symbolic)
%   .MpNE:      mass matrix
%   .bpNE:      coriolis/centrifugal
%   .gpNE:      gravity terms
%   .SpNE:      actuator selection matrix
%   .fpNE:      =SpNE*T;
%   .param:     input parameters
%   .q:         generalized coordinates
%   .Dq:        generalized velocities
%   .tau        generalized actuator forces
%
% * body.kin:   body contains (among others) now also global kinematics as:
%   .A_IB:      rotation matrix from B to I
%   .I_r_O:     vector inertia frame to body frame
%   .I_dr_O:    velocity of body frame in inertia frame
%   .I_r_CoG:   center of gravity position represented in inertia frame
%   .I_dr_CoG:  velcity ...
%   .I_J_CoG:   CoG jacobian
%   .I_Jr:      rotation jacobian
%   .I_Omega:   body rotational speed
%   ... and a lot more  
%
%
% -> calculates the symbolic equations of motion using projected Newton
% euler equations in the following form:
% M(q)*ddq + b(q,dq) + g(q,dq) = S' * f
% with 
% * M:      Massmatrix
% * q:      generalized coordinates
% * dq:
% * ddq:    acceleration in 
%
% For detailed information, see proNEu_documentation.pdf
%

function [sys, body, ftel] = computePNE(body,ftel,q,dq,tau,I_a_grav,param)

% make sure all vectors are columnvectors:
q(:,1) = q(:);
dq(:,1) = dq(:);
tau(:,1) = tau(:);
I_a_grav(:,1) = I_a_grav(:);param(:,1) = param(:);

% --------------------------------------------------------------------%
% add symbolic parameters and coordinates to sys
% --------------------------------------------------------------------%
sys.param = param;
sys.q = q;
sys.dq = dq;
sys.tau = tau;
% --------------------------------------------------------------------%
% create dynamic Object
% --------------------------------------------------------------------%

% isCalc: is needed such that we do not propagat always back to the root
isCalc = zeros(length(body),1);

ns = 4; % number of simplifications, 
% 100 is the default value of the simplify function but this takes very
% long for large systems.

for i=1:length(body)
    display(['computing dynamics of body ' num2str(i) '.'])
    
    % --------------------------------------------------------------------%
    % kinematics
    % --------------------------------------------------------------------%
    
    %  absolute position of origin of joint and rotation matrix
    % --------------------------------------------------------------------%
    body(i).kin.A_IB = body(i).cs.A_PB;
    body(i).kin.I_r_O = body(i).cs.P_r_PO;
    p = body(i).tree.parent;
    while (p ~= 0) % go until root
        if (isCalc(i))
            % update rotation matrix
            body(i).kin.A_IB = simplify(body(p).kin.A_IB*body(i).kin.A_IB,ns);
            % update position vectory to origin
            body(i).kin.I_r_O = simplify(body(p).cs.A_PB*body(i).kin.I_r_O + body(p).cs.P_r_PO + body(p).kin.I_r_O,ns);
            p = 0;
        else
            body(i).kin.A_IB = simplify(body(p).cs.A_PB*body(i).kin.A_IB,ns);
            body(i).kin.I_r_O = simplify(body(p).cs.A_PB*body(i).kin.I_r_O + body(p).cs.P_r_PO,ns);
            p = body(p).tree.parent;
        end
    end
    body(i).kin.A_BI = body(i).kin.A_IB';
    % derivative of rotation matrix
    body(i).kin.dA_IB =  dMATdt(body(i).kin.A_IB,q,dq);
    body(i).kin.dA_BI =  dMATdt(body(i).kin.A_BI,q,dq);
    
    % speed of origin
    body(i).kin.I_dr_O = dMATdt(sym(body(i).kin.I_r_O),q,dq);
    
    
    % center of gravity position and velocity
    % --------------------------------------------------------------------%
    body(i).kin.I_r_COG = simplify(body(i).kin.I_r_O + ...
                            body(i).kin.A_IB*body(i).param.B_r_COG,ns);
    body(i).kin.I_dr_COG = dMATdt(sym(body(i).kin.I_r_COG),q,dq);
   
    
    % Angular velocity in body (B) and inertial (I) frame
    % --------------------------------------------------------------------%
    body(i).kin.I_Omega_tilde = simplify(body(i).kin.dA_IB*body(i).kin.A_IB',ns);
    body(i).kin.I_Omega = unskew(body(i).kin.I_Omega_tilde);
    
    body(i).kin.B_Omega = simplify(body(i).kin.A_IB'*body(i).kin.I_Omega,ns);
    body(i).kin.B_Omega_tilde = skew(body(i).kin.B_Omega);
    
    
    % Jacobians
    % --------------------------------------------------------------------%
    % linear jacobians in inertial (I) frame
    body(i).kin.I_J_COG   = simplify((jacobian(body(i).kin.I_r_COG,q)),ns);
    body(i).kin.I_dJ_COG = dMATdt(body(i).kin.I_J_COG,q,dq);
    body(i).kin.I_J_O   = simplify((jacobian(body(i).kin.I_r_O,q)),ns);
    body(i).kin.I_dJ_O = dMATdt(body(i).kin.I_J_O,q,dq);
    body(i).kin.I_J_O   = simplify((jacobian(body(i).kin.I_r_O,q)),ns);
    body(i).kin.I_dJ_O = dMATdt(body(i).kin.I_J_O,q,dq);
    
    % transformation to body frame (B)
    body(i).kin.B_J_COG = simplify((body(i).kin.A_IB'*body(i).kin.I_J_COG),ns);
    body(i).kin.B_dJ_COG = simplify((body(i).kin.A_IB'*body(i).kin.I_dJ_COG),ns);
    body(i).kin.B_J_O = simplify((body(i).kin.A_IB'*body(i).kin.I_J_O),ns);
    body(i).kin.B_dJ_O = simplify((body(i).kin.A_IB'*body(i).kin.I_dJ_O),ns);
    
    % rotation jacobian in inertial (I) frame
    body(i).kin.I_Jr = simplify((jacobian(body(i).kin.I_Omega,dq)),ns);
    body(i).kin.I_dJr = dMATdt(body(i).kin.I_Jr,q,dq);
    
    % transformation to body frame (B)
    body(i).kin.B_Jr = simplify((body(i).kin.A_IB'*body(i).kin.I_Jr),ns);
    body(i).kin.B_dJr = simplify((body(i).kin.A_IB'*body(i).kin.I_dJr),ns);
    
    
    % --------------------------------------------------------------------%
    % gravity
    % --------------------------------------------------------------------%
    body(i).dyn.I_F_grav = body(i).param.m*I_a_grav;
    
    isCalc(i) = 1;
    
end

% --------------------------------------------------------------------%
% perform projected Newton-Euler
% --------------------------------------------------------------------%
% Mass matrix
display('computing MpNE matrix')
MpNE = sym(zeros(length(q)));
for i=1:length(body)
    MpNE = MpNE + body(i).kin.I_J_COG'*body(i).kin.I_J_COG*body(i).param.m + ...
        body(i).kin.B_Jr'*body(i).param.B_Th*body(i).kin.B_Jr;
end
MpNE = (MpNE);

% coriolis term
display('computing bpNE matrix')
bpNE = sym(zeros(size(q)));
for i=1:length(body)
    bpNE = bpNE + body(i).param.m*(body(i).kin.I_J_COG'*body(i).kin.I_dJ_COG*dq) + ...
        body(i).kin.B_Jr'*body(i).param.B_Th*body(i).kin.B_dJr*dq + ...
        body(i).kin.B_Jr'*body(i).kin.B_Omega_tilde*body(i).param.B_Th*body(i).kin.B_Omega;
end
bpNE = (bpNE);

% gravity
display('computing gpNE matrix')
gpNE = sym(zeros(size(q)));
for i=1:length(body)
    gpNE = gpNE - body(i).kin.I_J_COG'*body(i).dyn.I_F_grav;
end
gpNE = (gpNE);
            

% --------------------------------------------------------------------%
% generalized forces and torques
% --------------------------------------------------------------------%

display('computing fpNE matrix')
fpNE = sym(zeros(size(q)));
for i=1:length(ftel)
    switch(ftel(i).type)
        case('lin') % this is a linear force element
            
            % calculate jacobians in B frame
            ftel(i).B_J_A = body(ftel(i).body_B).kin.B_J_O + jacobian(ftel(i).B_r_A,q);
            
            % calculate generalized force of action
            ftel(i).fpNE = ftel(i).B_J_A'*ftel(i).B_F;
            
            if ftel(i).body_P ~=0
                % calculate jacobians in P frame
                ftel(i).P_J_R = body(ftel(i).body_P).kin.B_J_O + jacobian(ftel(i).P_r_R,q); 
                % transform force to P frame
                ftel(i).P_F = body(ftel(i).body_B).cs.A_PB*ftel(i).B_F;
                % calculate generalized force of reaction
                ftel(i).fpNE = ftel(i).fpNE + ...
                    ftel(i).P_J_R'*(-ftel(i).P_F);
            end
            
        case('rot') % this is a rotational force element
            % calculate torque of action
            ftel(i).fpNE = body(ftel(i).body_B).kin.B_Jr'*(ftel(i).B_T);
            if ftel(i).body_P ~=0
                % calculate torque of reaction
                ftel(i).fpNE = ftel(i).fpNE + ...
                    body(ftel(i).body_P).kin.B_Jr'*(-ftel(i).B_T);
            end
    end
    fpNE = fpNE + ftel(i).fpNE;
end


% --------------------------------------------------------------------%
% selection matrix
% --------------------------------------------------------------------%

display('computing SpNE matrix')
SpNE = jacobian(fpNE,tau)';

% update sys
sys.MpNE = MpNE;
sys.bpNE = bpNE;
sys.gpNE = gpNE;
sys.fpNE = fpNE;
sys.SpNE = SpNE;

% optionally make this simplification. can take very long for large systems
% sys.MpNE = simplify(MpNE);
% sys.bpNE = simplify(bpNE);
% sys.gpNE = simplify(gpNE);
% sys.fpNE = simplify(fpNE);
% sys.SpNE = simplify(SpNE);