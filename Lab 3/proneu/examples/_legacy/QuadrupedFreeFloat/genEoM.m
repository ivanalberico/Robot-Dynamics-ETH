% genEoM.m
% -> derivation of equations of motion for a free floating Quadruped
%
%       M(q)ddq + b(q,dq) + g(q) + Js(q)'*Fs = S'*T
%
% * M(q)    = Mass matrix
% * b(q,dq) = coriolis and centrifugal terms
% * g(q)    = gravitational terms
% * Js(q)   = ground contact (support) jacobian
% * Fs      = grouund contact (support) force
% * S       = actuator selection matrix
% * T       = acutator torque vector
%
% In this example file, these equations are derived 
% using projected Newton- Euler equations.
%
% The example is a quadruped robot with 3 DoF per leg
%
% proNEu: tool for symbolic EoM derivation
% Copyright (C) 2011  Marco Hutter, Christian Gehring
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
% Description of link object
% body(i).param.m         = mass [kg]
% body(i).param.B_Th      = moment of inertia w.r.t. center of gravity
%                         described in local coordinate system [kg*m^2]
% body(i).param.B_r_COG   = position of center of gravity w.r.t. origin of
%                         local frame described in local coordinate system
% body(i).cs.P_r_PO       = position of origin of local frame w.r.t.
%                         origin of parent frame described in parent 
%                         coordinate system
% body(i).cs.A_PB         = rotation matrix that rotates the parent frame to
%                         the local frame (3x3-matrix)
% body(i).kin.A_IB        = rotation matrix that rotates the local frame to
%                         the world frame (3x3-matrix)
% body(i).kin.dA_IB       = time derivative of A_IB (3x3-matrix)
% body(i).kin.A_BI
% body(i).kin.I_r_O      = position of origin of local frame described                        
%                         in world coordinate system
% body(i).kin.I_dr_O     = time derivative of I_r_O
% body(i).kin.I_r_COG    = position of center of gravity w.r.t. origin of
%                         world frame described in world coordinate system
% body(i).kin.I_dr_COG   = time derivative of I_r_COG
% body(i).kin.B_Omega    = angular velocity of joint  described 
%                         in world coordinate system
% body(i).kin.B_Omega_tilde  = angular velocity of joint in skew-symmetric 
%                         matrix described in world coordinate system
% body(i).kin.I_J_O      = Jacobian w.r.t. origin of local frame described
%                         in world coordinate system
% body(i).kin.I_dJ_O     = time derivative of I_J_O
% body(i).kin.I_J_COG    = Jacobian w.r.t. center of gravity described in
%                         world coordinate system
% body(i).kin.I_dJ_COG   = time derivative of I_J_COG
% body(i).kin.I_Jr       = rotational Jacobian described
%                         in world coordinate system
% body(i).kin.I_dJr      = time derivative of I_Jr
% body(i).dyn.B_F        = force vector described in local coord. system
% body(i).dyn.B_T        = torque vector described in local coord. system
% body(i).dyn.I_F_grav   = gravity force vector described in world c.s.
% body(i).tree.parent    = index of parent
%
% LF = left front 
% RF = right front
% LH = left hind
% RH = right hind
% HAA = hip abduction/ adduction
% HFE = hip flexion / extension
% KFE = knee flexion / extension

clc
clear 

%%%%%%%%%%%%%%%%%%%%%%
% Minimal coordinates
%%%%%%%%%%%%%%%%%%%%%%
syms qX qY qZ qAL qBE qGA real      % main body free floating
q = [qX qY qZ qAL qBE qGA]';
qDef = zeros(6,1);
syms qLF_HAA qLF_HFE qLF_KFE real   % LF (left front) leg
q = [q',qLF_HAA qLF_HFE qLF_KFE]';
qDef = [qDef', 0 pi/4 -pi/2]';
syms qRF_HAA qRF_HFE qRF_KFE real   % RF (right front) leg
q = [q',qRF_HAA qRF_HFE qRF_KFE]';
qDef = [qDef', 0 pi/4 -pi/2]';
syms qLH_HAA qLH_HFE qLH_KFE real   % LH (left hind) leg
q = [q',qLH_HAA qLH_HFE qLH_KFE]';
qDef = [qDef', 0 -pi/4 pi/2]';
syms qRH_HAA qRH_HFE qRH_KFE real   % RH (right hind) leg
q = [q',qRH_HAA qRH_HFE qRH_KFE]';
qDef = [qDef', 0 -pi/4 pi/2]';

% velocity
syms DqX DqY DqZ DqAL DqBE DqGA real % main body velocity
Dq = [DqX DqY DqZ DqAL DqBE DqGA]';
syms DqLF_HAA DqLF_HFE DqLF_KFE real
Dq = [Dq',DqLF_HAA DqLF_HFE DqLF_KFE]';
syms DqRF_HAA DqRF_HFE DqRF_KFE real 
Dq = [Dq',DqRF_HAA DqRF_HFE DqRF_KFE]';
syms DqLH_HAA DqLH_HFE DqLH_KFE real 
Dq = [Dq',DqLH_HAA DqLH_HFE DqLH_KFE]';
syms DqRH_HAA DqRH_HFE DqRH_KFE real 
Dq = [Dq',DqRH_HAA DqRH_HFE DqRH_KFE]';
DqDef = zeros(size(Dq));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Parameters 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
syms mB sB ThB_xx ThB_yy ThB_zz real 
param = [mB sB ThB_xx ThB_yy ThB_zz]';
paramDef = [1 0 1 1 1]';
syms mH sH lH ThH_xx ThH_yy ThH_zz real 
param = [param',mH sH lH ThH_xx ThH_yy ThH_zz]';
paramDef = [paramDef',1 -0.01 -0.02 1 1 1]'; 
syms mT sT lT ThT_xx ThT_yy ThT_zz real 
param = [param',mT sT lT ThT_xx ThT_yy ThT_zz]';
paramDef = [paramDef',1 -0.1 -0.2 1 1 1]';
syms mS sS lS ThS_xx ThS_yy ThS_zz real 
param = [param',mS sS lS ThS_xx ThS_yy ThS_zz]';
paramDef = [paramDef',1 -0.1 -0.2 1 1 1]';
syms b2hx b2hy real
param = [param',b2hx b2hy]';
paramDef = [paramDef',0.2 0.15]';
syms g real;
param = [param',g]';
paramDef = [paramDef',9.81]';


%%%%%%%%%%%%%%%%%%%%%%
% Torques
%%%%%%%%%%%%%%%%%%%%%%
syms TLF_HAA TLF_HFE TLF_KFE real   % LF torques
T = [TLF_HAA TLF_HFE TLF_KFE]';
syms TRF_HAA TRF_HFE TRF_KFE real   % RF torques
T = [T',TRF_HAA TRF_HFE TRF_KFE]';
syms TLH_HAA TLH_HFE TLH_KFE real   % LH torques
T = [T',TLH_HAA TLH_HFE TLH_KFE]';
syms TRH_HAA TRH_HFE TRH_KFE real   % RH torques
T = [T', TRH_HAA TRH_HFE TRH_KFE]';

%%%%%%%%%%%%%%%%%%%%%%
% Gravity
%%%%%%%%%%%%%%%%%%%%%%
% gravitational constant
I_a_grav = [0; 0; -g];


%% DYNAMICS

% Main Body
i = 1;
body(i).param.m = mB;
body(i).param.B_Th = sym(diag([ThB_xx ThB_yy ThB_zz]));
body(i).param.B_r_COG = sym([0;0;sB]);
body(i).cs.P_r_PO = sym([qX;qY;qZ]); 
body(i).cs.A_PB = eulerToRotMat_A_IB(qAL,qBE,qGA);
body(i).tree.parent = 0;

% LF_HAA
i = i+1;
body(i).param.m = mH;
body(i).param.B_Th = sym(diag([ThH_xx ThH_yy ThH_zz]));
body(i).param.B_r_COG = [0;0;sH];
body(i).cs.P_r_PO = sym([b2hx;b2hy;0]);
body(i).cs.A_PB = eulerToRotMat_A_IB(qLF_HAA,0,0);
body(i).tree.parent = 1;

% LF_HFE
i = i+1;
body(i).param.m = mT;
body(i).param.B_Th = sym(diag([ThT_xx ThT_yy ThT_zz]));
body(i).param.B_r_COG = [0;0;sT];
body(i).cs.P_r_PO = sym([0;0;lH]);
body(i).cs.A_PB = eulerToRotMat_A_IB(0,qLF_HFE,0);
body(i).tree.parent = i-1;

% LF_KFE
i = i+1;
body(i).param.m = mS;
body(i).param.B_Th = sym(diag([ThS_xx ThS_yy ThS_zz]));
body(i).param.B_r_COG = [0;0;sS];
body(i).cs.P_r_PO = sym([0;0;lT]);
body(i).cs.A_PB = eulerToRotMat_A_IB(0,qLF_KFE,0);
body(i).tree.parent = i-1;

% LF_Foot
i = i+1;
body(i).param.m = 0;
body(i).param.B_Th = diag([0, 0, 0]);
body(i).param.B_r_COG = [0;0;0];
body(i).cs.P_r_PO = sym([0;0;lS]);
body(i).cs.A_PB = eulerToRotMat_A_IB(0,0,0);
body(i).tree.parent = i-1;

% RF_HAA
i = i+1;
body(i).param.m = mH;
body(i).param.B_Th = sym(diag([ThH_xx ThH_yy ThH_zz]));
body(i).param.B_r_COG = [0;0;sH];
body(i).cs.P_r_PO = sym([b2hx;-b2hy;0]);
body(i).cs.A_PB = eulerToRotMat_A_IB(qRF_HAA,0,0);
body(i).tree.parent = 1;

% RF_HFE
i = i+1;
body(i).param.m = mT;
body(i).param.B_Th = sym(diag([ThT_xx ThT_yy ThT_zz]));
body(i).param.B_r_COG = [0;0;sT];
body(i).cs.P_r_PO = sym([0;0;lH]);
body(i).cs.A_PB = eulerToRotMat_A_IB(0,qRF_HFE,0);
body(i).tree.parent = i-1;

% RF_KFE
i = i+1;
body(i).param.m = mS;
body(i).param.B_Th = sym(diag([ThS_xx ThS_yy ThS_zz]));
body(i).param.B_r_COG = [0;0;sS];
body(i).cs.P_r_PO = sym([0;0;lT]);
body(i).cs.A_PB = eulerToRotMat_A_IB(0,qRF_KFE,0);
body(i).tree.parent = i-1;

% RF_Foot
i = i+1;
body(i).param.m = 0;
body(i).param.B_Th = diag([0, 0, 0]);
body(i).param.B_r_COG = [0;0;0];
body(i).cs.P_r_PO = sym([0;0;lS]);
body(i).cs.A_PB = eulerToRotMat_A_IB(0,0,0);
body(i).tree.parent = i-1;

% LH_HAA
i = i+1;
body(i).param.m = mH;
body(i).param.B_Th = sym(diag([ThH_xx ThH_yy ThH_zz]));
body(i).param.B_r_COG = [0;0;sH];
body(i).cs.P_r_PO = sym([-b2hx;b2hy;0]);
body(i).cs.A_PB = eulerToRotMat_A_IB(qLH_HAA,0,0);
body(i).tree.parent = 1;

% LH_HFE
i = i+1;
body(i).param.m = mT;
body(i).param.B_Th = sym(diag([ThT_xx ThT_yy ThT_zz]));
body(i).param.B_r_COG = [0;0;sT];
body(i).cs.P_r_PO = sym([0;0;lH]);
body(i).cs.A_PB = eulerToRotMat_A_IB(0,qLH_HFE,0);
body(i).tree.parent = i-1;

% LH_KFE
i = i+1;
body(i).param.m = mS;
body(i).param.B_Th = sym(diag([ThS_xx ThS_yy ThS_zz]));
body(i).param.B_r_COG = [0;0;sS];
body(i).cs.P_r_PO = sym([0;0;lT]);
body(i).cs.A_PB = eulerToRotMat_A_IB(0,qLH_KFE,0);
body(i).tree.parent = i-1;

% LH_Foot
i = i+1;
body(i).param.m = 0;
body(i).param.B_Th = diag([0, 0, 0]);
body(i).param.B_r_COG = [0;0;0];
body(i).cs.P_r_PO = sym([0;0;lS]);
body(i).cs.A_PB = eulerToRotMat_A_IB(0,0,0);
body(i).tree.parent = i-1;

% RH_HAA
i = i+1;
body(i).param.m = mH;
body(i).param.B_Th = sym(diag([ThH_xx ThH_yy ThH_zz]));
body(i).param.B_r_COG = [0;0;sH];
body(i).cs.P_r_PO = sym([-b2hx;-b2hy;0]);
body(i).cs.A_PB = eulerToRotMat_A_IB(qRH_HAA,0,0);
body(i).tree.parent = 1;

% RH_HFE
i = i+1;
body(i).param.m = mT;
body(i).param.B_Th = sym(diag([ThT_xx ThT_yy ThT_zz]));
body(i).param.B_r_COG = [0;0;sT];
body(i).cs.P_r_PO = sym([0;0;lH]);
body(i).cs.A_PB = eulerToRotMat_A_IB(0,qRH_HFE,0);
body(i).tree.parent = i-1;

% RH_KFE
i = i+1;
body(i).param.m = mS;
body(i).param.B_Th = sym(diag([ThS_xx ThS_yy ThS_zz]));
body(i).param.B_r_COG = [0;0;sS];
body(i).cs.P_r_PO = sym([0;0;lT]);
body(i).cs.A_PB = eulerToRotMat_A_IB(0,qRH_KFE,0);
body(i).tree.parent = i-1;

% RH_Foot
i = i+1;
body(i).param.m = 0;
body(i).param.B_Th = diag([0, 0, 0]);
body(i).param.B_r_COG = [0;0;0];
body(i).cs.P_r_PO = sym([0;0;lS]);
body(i).cs.A_PB = eulerToRotMat_A_IB(0,0,0);
body(i).tree.parent = i-1;

%%%%%%%%%%%%%%%%%%%%%%%%
% Force/torque elements
%%%%%%%%%%%%%%%%%%%%%%%%

% Torque at FL 
i = 1;
ftel(i).type = 'rot';       % define it to be a rotational = torque 
ftel(i).body_P = 1;         % body on which the reaction happens
ftel(i).body_B = 2;         % body on which the action happens
ftel(i).B_T = [TLF_HAA;0;0];     % torque vector, expressed in B frame

i = i+1;
ftel(i).type = 'rot';       
ftel(i).body_P = 2;         
ftel(i).body_B = 3;         
ftel(i).B_T = [0;TLF_HFE;0];     

i = i+1;
ftel(i).type = 'rot';       
ftel(i).body_P = 3;         
ftel(i).body_B = 4;         
ftel(i).B_T = [0;TLF_KFE;0];

% Torque at FR
i = i+1;
ftel(i).type = 'rot';       
ftel(i).body_P = 1;         
ftel(i).body_B = 6;         
ftel(i).B_T = [TRF_HAA;0;0];   

i = i+1;
ftel(i).type = 'rot';       
ftel(i).body_P = 6;         
ftel(i).body_B = 7;         
ftel(i).B_T = [0;TRF_HFE;0];     

i = i+1;
ftel(i).type = 'rot';       
ftel(i).body_P = 7;         
ftel(i).body_B = 8;         
ftel(i).B_T = [0;TRF_KFE;0];

% Torque at LH
i = i+1;
ftel(i).type = 'rot';       
ftel(i).body_P = 1;         
ftel(i).body_B = 10;         
ftel(i).B_T = [TLH_HAA;0;0];   

i = i+1;
ftel(i).type = 'rot';       
ftel(i).body_P = 10;         
ftel(i).body_B = 11;         
ftel(i).B_T = [0;TLH_HFE;0];     

i = i+1;
ftel(i).type = 'rot';       
ftel(i).body_P = 11;         
ftel(i).body_B = 12;         
ftel(i).B_T = [0;TLH_KFE;0];

% Torque at RH
i = i+1;
ftel(i).type = 'rot';       
ftel(i).body_P = 1;         
ftel(i).body_B = 14;         
ftel(i).B_T = [TRH_HAA;0;0];   

i = i+1;
ftel(i).type = 'rot';       
ftel(i).body_P = 14;         
ftel(i).body_B = 15;         
ftel(i).B_T = [0;TRH_HFE;0];     

i = i+1;
ftel(i).type = 'rot';       
ftel(i).body_P = 15;         
ftel(i).body_B = 16;         
ftel(i).B_T = [0;TRH_KFE;0];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Projected Newton Euler equations
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[sys, body, ftel] = computePNE(body,ftel,q,Dq,T,I_a_grav,param);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Visualization
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% plot StarlETH
plotBodies(body,param,paramDef,q,qDef);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Get ground contact jacobians
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% this would be the solution if all legs were at the ground
Js = [...
    body(5).kin.I_J_O;...
    body(9).kin.I_J_O;...
    body(13).kin.I_J_O;...
    body(17).kin.I_J_O];