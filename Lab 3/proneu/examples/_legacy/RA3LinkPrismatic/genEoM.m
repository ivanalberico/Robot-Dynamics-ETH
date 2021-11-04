% 3LinkRobotArmPrismaticLink_genEoM.m
%
% -> exmaple file to generate the EoM of a 3-link robot arm that has a
% prismatic joint as the last actuator.  For detailed information and a
% figure with the links and coordinate systems, please have a look at the
% documentation.
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

clc
clear all

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Minimal coordinates and velocity
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% => define in this section the minimal coordinates that you
% want to use. They can stand for rotational or prismatic joints.

syms q1 q2 x real               % generalized coordinates
q = [q1 q2 x]';                 % q1, q2: 'rot'    x:'lin'
qDef = [pi/4,pi/4,0.2]';        % define here IC for visualization

syms Dq1 Dq2 Dx real            % generalized velocities
dq = [Dq1 Dq2 Dx]';
dqDef = [0,0,0]';

%%%%%%%%%%%%%%%%%
% Torques/Forces
%%%%%%%%%%%%%%%%%
% => define in this section the generalized forces (for prismatic joints)
% and the generalized torques (for rotational joints)

syms T1 T2 F1 real              % generalized Force/Torque vector
T = [T1 T2 F1]';
TDef = [1,2,3]';


%%%%%%%%%%%%%%%%%%%%%%
% Parameters
%%%%%%%%%%%%%%%%%%%%%%
% => define in this section the parameters to describe the link properties
% i.e.      l1: length to next joint
%           s1: offset CoG 
%           m1: link mass
%           Th1.. : inertia properties of link$

syms l1 s1 m1 Th1_xx Th1_yy Th1_zz real
syms l2 s2 m2 Th2_xx Th2_yy Th2_zz real  
syms l3 s3 m3 Th3_xx Th3_yy Th3_zz real  
param = [l1 s1 m1 Th1_xx Th1_yy Th1_zz]';       % this defines the order
param = [param', l2 s2 m2 Th2_xx Th2_yy Th2_zz]';
param = [param', l3 s3 m3 Th3_xx Th3_yy Th3_zz]';
paramDef = [1 0.5 1 1 1 1]';          % define here the default values
paramDef = [paramDef', 1 0.5 1 1 1 1]';
paramDef = [paramDef', 1 0.5 1 1 1 1]';


%%%%%%%%%%%%%%%%%%%%%%
% Gravity
%%%%%%%%%%%%%%%%%%%%%%
syms g real
I_a_grav = [0; 0; -g];
param = [param;g];
paramDef = [paramDef;9.81];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Kinematic structure - kinematic tree
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% => start setting up the kinematic structure.  
% Each link needs to have all the folowing struct elements 
% B indicates body frame
% P indicates coordinate system of parent element

% Body 1
i = 1;
body(i).param.m = m1;           % mass
body(i).param.B_Th = diag([Th1_xx Th1_yy Th1_zz]);  % inertia tensor
body(i).param.B_r_COG = [0;0;s1]; % center of gravity in body frame
body(i).cs.P_r_PO = sym([0;0;0]);  % position of origin in parent CS
body(i).cs.A_PB = eulerToRotMat_A_IB(0,0,q1); % rotation from parent CS
body(i).tree.parent = 0;    % tree parent (0=inertial frame)
    
% Body 2
i = 2;
body(i).param.m = m2;
body(i).param.B_Th = diag([Th2_xx Th2_yy Th2_zz]);
body(i).param.B_r_COG = [s2;0;0];
body(i).cs.P_r_PO = [0;0;l1]; 
body(i).cs.A_PB = eulerToRotMat_A_IB(0,q2,0);
body(i).tree.parent = 1;

% Body 3
i = 3;
body(i).param.m = m3;
body(i).param.B_Th = diag([Th3_xx Th3_yy Th3_zz]);
body(i).param.B_r_COG = [s3;0;0];
body(i).cs.P_r_PO = [l2+x;0;0]; 
body(i).cs.A_PB = eulerToRotMat_A_IB(0,0,0);
body(i).tree.parent = 2;


%%%%%%%%%%%%%%%%%%%%%%%%
% Force/torque elements
%%%%%%%%%%%%%%%%%%%%%%%%
% => define in this section the force / torque elements that act 
% between the bodies

% torque element acting on body 1
i = 1;
ftel(i).type = 'rot';       % define it to be a rotational = torque 
ftel(i).body_P = 0;         % body on which the reaction happens
ftel(i).body_B = 1;         % body on which the action happens
ftel(i).B_T = [0;0;T1];     % torque vector, expressed in B frame

% torque element acting between body 1 and body 2
i = 2;
ftel(i).type = 'rot';       % define it to be a rotational = torque 
ftel(i).body_P = 1;         % body on which the reaction happens
ftel(i).body_B = 2;         % body on which the action happens
ftel(i).B_T = [0;T2;0];     % torque vector, expressed in B frame

% force element acting between body 2 and body 3
i = 3;
ftel(i).type = 'lin';           % define it to be a linear = force 
ftel(i).body_P = 2;             % body on which the reaction happens
ftel(i).body_B = 3;             % body on which the action happens
ftel(i).P_r_R = sym([0;0;0]);   % point of reaction in P frame
ftel(i).B_r_A = sym([0;0;0]);   % point of action in B frame
ftel(i).B_F = [F1;0;0];         % force vector of action / this is optional


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Projected Newton Euler equations
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[sys, body, ftel] = computePNE(body,ftel,q,dq,T,I_a_grav,param);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Save the data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% save the kinematics and dynamics
save('body','body')
save('sys','sys')
% save the numerical values in a struct
values.paramDef = paramDef;
values.qDef = qDef;
values.dqDef = dqDef;
values.TDef = TDef;
save('values','values')
% save the force/torque elements
save('ftel','ftel')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Visualization
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
plotBodies(body,param,paramDef,q,qDef);
