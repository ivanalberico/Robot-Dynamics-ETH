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
%   File:           robotarm_dynamics.m
%
%   Authors:        Vassilios Tsounis, tsounisv@ethz.ch
%
%   Date:           28/9/2016
%
%   Desciption:     Analytical derivation of the EoM for an under-actuated
%                   3-Dof robot arm.
%

% Optional calls for workspace resetting/cleaning
close all; clear all; clear classes; clc;

%% World Gravity

% Gravitational potential field in inertial world-frame
syms grav real;
e_I_g = [0 0 1].';
I_a_g = grav*e_I_g;

%% System Parameters & Variables

% Declare all system parametes
syms m_B m_1 m_2 m_3 real;

syms d_Bx d_By d_Bz real;
syms l_B w_B h_B real;

syms d_1x d_1y d_1z real;
syms r_1 l_1 real;

syms d_2x d_2y d_2z real;
syms r_2 l_2 real;

syms d_3x d_3y d_3z real;
syms r_3 l_3 real;

% Declare all system variables
syms phi_1 phi_2 phi_3 real;

% Declare all joint torque variables
syms tau_1 tau_2 tau_3 real;

% Declare all external force variables at the end-effector
syms F_EEx F_EEy F_EEz real;

%% Multi-Body System Description

% Base
i=1;
body(i)                     = RigidBodyDescription_v2;
body(i).name                = 'B';
body(i).ktree.nodeid        = i;
body(i).ktree.parents       = i-1;
body(i).cs.P_r_PB           = [0 0 0].';
body(i).cs.C_PB             = sym(eye(3));
body(i).param.m             = m_B;
body(i).param.B_Theta       = [];
body(i).param.B_r_BCoM      = [d_Bx d_By d_Bz].'; 
body(i).param.C_BCoM        = sym(eye(3));
body(i).geometry.type       = 'cuboid';
body(i).geometry.issolid    = true;
body(i).geometry.params     = [l_B w_B h_B] ; 
body(i).geometry.values     = [0.3 0.3 0.2];
body(i).geometry.offsets.r  = [0 0 -0.1].';
body(i).geometry.offsets.C  = eye(3);
body(i).geometry.color      = [197 172 202]/255;

% First link
i=2;
body(i)                     = RigidBodyDescription_v2;
body(i).name                = 'L1';
body(i).ktree.nodeid        = i;
body(i).ktree.parents       = i-1;
body(i).cs.P_r_PB           = [0 0 h_B].';
body(i).cs.C_PB             = getRotationMatrixZ(phi_1);
body(i).param.m             = m_1;
body(i).param.B_Theta       = [];
body(i).param.B_r_BCoM      = [d_1x d_1y d_1z].'; 
body(i).param.C_BCoM        = sym(eye(3));
body(i).geometry.type       = 'cylinder';
body(i).geometry.issolid    = true;
body(i).geometry.params     = [r_1 l_1]; 
body(i).geometry.values     = [0.07 0.4];
body(i).geometry.offsets.r  = [0 0 -0.2].';
body(i).geometry.offsets.C  = eye(3);
body(i).geometry.color      = [0.6 0.6 0.6];

% Second link
i=3;
body(i)                     = RigidBodyDescription_v2;
body(i).name                = 'L2';
body(i).ktree.nodeid        = i;
body(i).ktree.parents       = i-1;
body(i).cs.P_r_PB           = [0 0 l_1].';
body(i).cs.C_PB             = getRotationMatrixY(phi_2);
body(i).param.m             = m_2;
body(i).param.B_Theta       = [];
body(i).param.B_r_BCoM      = [d_2x d_2y d_2z].'; 
body(i).param.C_BCoM        = sym(eye(3));
body(i).geometry.type       = 'cylinder';
body(i).geometry.issolid    = true;
body(i).geometry.params     = [r_2 l_2]; 
body(i).geometry.values     = [0.05 1.0];
body(i).geometry.offsets.r  = [0 0 -0.5].';
body(i).geometry.offsets.C  = eye(3);
body(i).geometry.color      = [0.6 0.6 0.6];

% Third link
i=4;
body(i)                     = RigidBodyDescription_v2;
body(i).name                = 'L3';
body(i).ktree.nodeid        = i;
body(i).ktree.parents       = i-1;
body(i).cs.P_r_PB           = [0 0 l_2].';
body(i).cs.C_PB             = getRotationMatrixY(phi_3);
body(i).param.m             = m_3;
body(i).param.B_Theta       = [];
body(i).param.B_r_BCoM      = [d_3x d_3y d_3z].'; 
body(i).param.C_BCoM        = sym(eye(3));
body(i).geometry.type       = 'cylinder';
body(i).geometry.issolid    = true;
body(i).geometry.params     = [r_3 l_3]; 
body(i).geometry.values     = [0.05 1.0];
body(i).geometry.offsets.r  = [0 0 -0.5].';
body(i).geometry.offsets.C  = eye(3);
body(i).geometry.color      = [0.6 0.6 0.6];

% End-Effector
i=5;
body(i)                     = RigidBodyDescription_v2;
body(i).name                = 'EE';
body(i).ktree.nodeid        = i;
body(i).ktree.parents       = i-1;
body(i).cs.P_r_PB           = [0 0 l_3].';
body(i).cs.C_PB             = sym(eye(3));

%% Definition of External Forces & Torques

% First actuator
j=1;
ftel(j) = ForceTorqueDescription_v2;
ftel(j).name    = 'tau_1';
ftel(j).type    = 'rotational';
ftel(j).body_P  = 1; 
ftel(j).body_B  = 2;
ftel(j).B_T     = [0 0 tau_1].';

% Second actuator
j=2;
ftel(j) = ForceTorqueDescription_v2;
ftel(j).name    = 'tau_2';
ftel(j).type    = 'rotational';
ftel(j).body_P  = 2; 
ftel(j).body_B  = 3;
ftel(j).B_T     = [0 tau_2 0].';

% Third actuator
j=3;
ftel(j) = ForceTorqueDescription_v2;
ftel(j).name    = 'tau_3';
ftel(j).type    = 'rotational';
ftel(j).body_P  = 3; 
ftel(j).body_B  = 4;
ftel(j).B_T     = [0 tau_3 0].';

j=4;
ftel(j) = ForceTorqueDescription_v2;
ftel(j).name    = 'F_EE';
ftel(j).type    = 'linear';
ftel(j).body_P  = 0; 
ftel(j).body_B  = 5;
ftel(j).P_r_R   = sym([0 0 0].');
ftel(j).B_r_A   = sym([0 0 0].');
ftel(j).I_F     = [F_EEx F_EEy F_EEz].';

%% System Definitions

% Definition of the joint DoFs of 2-link system
q_j  = [phi_1 phi_2 phi_3].';

% Controllable joint forces/torques
tau_j = [tau_1 tau_2 tau_3].';

% External forces/torques
tau_env = [F_EEx F_EEy F_EEz].';

%% Generate Full System Model using proNEu.v2

% Give a name to the model
robot_name = 'RobotArmModel';

% Generate the model object
robotmdl = RobotModel(body, ftel, q_j, tau_j, tau_env, I_a_g, 'name', robot_name, 'type', 'fixed', 'method', 'proneu', 'symsteps', 100);

%% Generate MATLAB numerical functions

% Generate numerical functions
robotmdl.generatefunctions();

%% Save to MAT File

% Generate file and directory paths
fname = mfilename;
fpath = mfilename('fullpath');
dpath = strrep(fpath, fname, '');

% Store generated model in the appropriate directory
save(strcat(dpath,robotmdl.name), 'robotmdl');

%% 
% EOF
