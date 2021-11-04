% genEoM.m
%
% -> generation of the EoM for a robot arm with three links.
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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Minimal coordinates and derivatives 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% => define in this section the minimal coordinates that you
% want to use. They can stand for rotational or prismatic joints.

syms q1 q2 q3 real           % define the three angles
q = [q1 q2 q3]';             % vector of generalized coordinates
qDef = [pi/4,pi/4, pi/4]';   % define some values for visualization

syms Dq1 Dq2 Dq3 real        % define the derivatives of the angles
dq = [Dq1 Dq2 Dq3]';         % derivatives of the gen. cord.
dqDef = [0,0,0]';            % define some values

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Generalized actuator torques
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% => define in this section the generalized forces (for prismatic joints)
% and the generalized torques (for rotational joints)
syms T1 T2 T3 real          % define the three torques
T = [T1 T2 T3]';            % torque vector
TDef = [1,2,3]';            % define some values



%%%%%%%%%%%%%%%%%%%%%%
% Parameters
%%%%%%%%%%%%%%%%%%%%%%
% => define in this section the parameters to describe 
% the link properties
% i.e.      l1: length to next joint
%           s1: offset CoG 
%           m1: link mass
%           Th1.. : inertia properties of link$

syms l1 s1 m1 Th1_xx Th1_yy Th1_zz real                 % link 1
syms l2 s2 m2 Th2_xx Th2_yy Th2_zz real                 % link 2
syms l3 s3 m3 Th3_xx Th3_yy Th3_zz real                 % link 3
% parameter vector
param = [l1 s1 m1 Th1_xx Th1_yy Th1_zz]';               % link 1
param = [param', l2 s2 m2 Th2_xx Th2_yy Th2_zz]';       % link 2
param = [param', l3 s3 m3 Th3_xx Th3_yy Th3_zz]';       % link 3
 % define some values for visualization
paramDef = [1 0.5 1 1 1 1]';                            % link 1
paramDef = [paramDef', 1 0.5 1 1 1 1]';                 % link 2
paramDef = [paramDef', 1 0.5 1 1 1 1]';                 % link 3

%%%%%%%%%%%%%%%%%%%%%%
% Gravity
%%%%%%%%%%%%%%%%%%%%%%
syms g real
I_a_grav = [0; 0; -g];         % gravity vector expressed in inertial frame
param = [param;g];             % add gravity to the parameter vector
paramDef = [paramDef;9.81];    % define the value

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Kinematic structure - kinematic tree
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% => start setting up the kinematic structure.  
% Each link needs to have all the folowing struct elements 
% B indicates body frame
% P indicates coordinate system of parent element

% Body 1
i = 1;
body(i).param.m = m1; % mass
body(i).param.B_Th = sym(diag([Th1_xx Th1_yy Th1_zz])); % inertia in B frame
body(i).param.B_r_COG = sym([0;0;s1]); % CoG position in B frame
body(i).cs.P_r_PO = sym([0;0;0]); % body frame origin in predecessor frame
body(i).cs.A_PB = eulerToRotMat_A_IB(0,0,q1); % rotation of body frame
body(i).tree.parent = 0; % parent element
    
% Body 2
i = 2;
body(i).param.m = m2;
body(i).param.B_Th = sym(diag([Th2_xx Th2_yy Th2_zz]));
body(i).param.B_r_COG = sym([0;0;s2]);
body(i).cs.P_r_PO = sym([0;0;l1]); 
body(i).cs.A_PB = eulerToRotMat_A_IB(q2,0,0);
body(i).tree.parent = 1;

% Body 3
i = 3;
body(i).param.m = m3;
body(i).param.B_Th = sym(diag([Th3_xx Th3_yy Th3_zz]));
body(i).param.B_r_COG = sym([0;0;s3]);
body(i).cs.P_r_PO = sym([0;0;l2]); 
body(i).cs.A_PB = eulerToRotMat_A_IB(0,q3,0);
body(i).tree.parent = 2;

%%%%%%%%%%%%%%%%%%%%%%%%
% Force/torque elements
%%%%%%%%%%%%%%%%%%%%%%%%
% => define in this section the force / torque elements that act 
% between the bodies

% torque element acting on body 1
ftel(1).type = 'rot';       % define it to be a rotational = torque 
ftel(1).body_P = 0;         % body on which the reaction happens
ftel(1).body_B = 1;         % body on which the action happens
ftel(1).B_T = [0;0;T1];     % torque vector, expressed in B frame

% torque element acting between body 1 and body 2
ftel(2).type = 'rot';       % define it to be a rotational = torque 
ftel(2).body_P = 1;         % body on which the reaction happens
ftel(2).body_B = 2;         % body on which the action happens
ftel(2).B_T = [T2;0;0];     % torque vector, expressed in B frame

% torque element acting between body 2 and body 3
ftel(3).type = 'rot';       % define it to be a rotational = torque 
ftel(3).body_P = 2;         % body on which the reaction happens
ftel(3).body_B = 3;         % body on which the action happens
ftel(3).B_T = [0;T3;0];     % torque vector, expressed in B frame

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Projected Newton Euler equations
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% => compute the dynamics by applying the proj. Newton-Euler method 
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
% plot the robot arm
plotBodies(body,param,paramDef,q,qDef);
