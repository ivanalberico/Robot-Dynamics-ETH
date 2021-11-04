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
%   File:           quadrotor_dynamics.m
%
%   Authors:        Vassilios Tsounis, tsounisv@ethz.ch
%
%   Date:           28/9/2016
%
%   Desciption:     Analytical derivation of the EoM for a quadrotor with
%                   geometric simplifications.
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
syms m_B m_A m_H m_R real;

% Body (fuselage)
syms d_Bx d_By d_Bz real;
syms l_B w_B h_B real;

% Rotor arms
syms d_Ax d_Ay d_Az real;
syms r_A l_A real;

% Rotor hubs
syms d_Hx d_Hy d_Hz real;
syms r_H l_H real;

% Rotor discs
syms d_Rx d_Ry d_Rz real;
syms r_R l_R real;

% Declare all system variables
syms phi_1 phi_2 phi_3 phi_4 real;
syms F_T1 F_T2 F_T3 F_T4  real;
syms T_Q1 T_Q2 T_Q3 T_Q4  real;
syms F_Ex F_Ey F_Ez T_Ex T_Ey T_Ez real;

%% Multi-Body System Description

% Fuselage body
i=1;
body(i)                     = RigidBodyDescription_v2;
body(i).name                = 'B';
body(i).ktree.nodeid        = i;
body(i).ktree.parents       = i-1;
body(i).cs.P_r_PB           = [];
body(i).cs.C_PB             = [];
body(i).param.m             = m_B;
body(i).param.B_Theta_B     = [];
body(i).param.B_r_BCoM      = [0 0 0].'; 
body(i).param.C_BCoM        = sym(eye(3));
body(i).geometry.type       = 'cuboid';
body(i).geometry.issolid    = true;
body(i).geometry.params     = [l_B w_B h_B]; 
body(i).geometry.values     = [0.15 0.15 0.08];
body(i).geometry.offsets.r  = [0 0 0].';
body(i).geometry.offsets.C  = eye(3);
body(i).geometry.color      = [197 172 202]/255;

% Rotor Arm 1
i=2;
body(i)                     = RigidBodyDescription_v2;
body(i).name                = 'none';
body(i).ktree.nodeid        = 2;
body(i).ktree.parents       = 1;
body(i).cs.P_r_PB           = [l_B/2 0 0].';
body(i).cs.C_PB             = getRotationMatrixZ(0)*getRotationMatrixY(pi/2);
body(i).param.m             = m_A;
body(i).param.B_Theta_B     = [];
body(i).param.B_r_BCoM      = [0 0 d_Az].'; 
body(i).param.C_BCoM        = sym(eye(3));
body(i).geometry.type       = 'cylinder';
body(i).geometry.issolid    = true;
body(i).geometry.params     = [r_A l_A]; 
body(i).geometry.values     = [0.015 0.4];
body(i).geometry.offsets.r  = [0 0 -0.2].';
body(i).geometry.offsets.C  = eye(3);
body(i).geometry.color      = [0.6 0.6 0.6];

% Rotor Hub 1
i=3;
body(i)                     = RigidBodyDescription_v2;
body(i).name                = 'none';
body(i).ktree.nodeid        = 3;
body(i).ktree.parents       = 2;
body(i).cs.P_r_PB           = [0 0 l_A].';
body(i).cs.C_PB             = getRotationMatrixY(-pi/2);
body(i).param.m             = m_H;
body(i).param.B_Theta_B     = [];
body(i).param.B_r_BCoM      = [0 0 0].'; 
body(i).param.C_BCoM        = sym(eye(3));
body(i).geometry.type       = 'cylinder';
body(i).geometry.issolid    = true;
body(i).geometry.params     = [r_H l_H]; 
body(i).geometry.values     = [0.03 0.05];
body(i).geometry.offsets.r  = [0 0 0].';
body(i).geometry.offsets.C  = eye(3);
body(i).geometry.color      = [0.6 0.6 0.6];

% Rotor Disc 1
i=4;
body(i)                     = RigidBodyDescription_v2;
body(i).name                = 'none';
body(i).ktree.nodeid        = 4;
body(i).ktree.parents       = 3;
body(i).cs.P_r_PB           = [0 0 l_H/2].';
body(i).cs.C_PB             = sym(eye(3));
body(i).param.m             = m_R;
body(i).param.B_Theta_B     = [];
body(i).param.B_r_BCoM      = [0 0 0].';
body(i).param.C_BCoM        = eye(3);
body(i).geometry.type       = 'cylinder';
body(i).geometry.issolid    = true;
body(i).geometry.params     = [r_R l_R]; 
body(i).geometry.values     = [0.12 0.005];
body(i).geometry.offsets.r  = [0 0 0].';
body(i).geometry.offsets.C  = eye(3);
body(i).geometry.color      = [0.6 0.6 0.8];

% Rotor Arm 2
i=5;
body(i)                     = RigidBodyDescription_v2;
body(i).name                = 'none';
body(i).ktree.nodeid        = i;
body(i).ktree.parents       = 1;
body(i).cs.P_r_PB           = [0 -w_B/2 0].';
body(i).cs.C_PB             = getRotationMatrixZ(-pi/2)*getRotationMatrixY(pi/2);
body(i).param.m             = m_A;
body(i).param.B_Theta_B     = [];
body(i).param.B_r_BCoM      = [0 0 d_Az].'; 
body(i).param.C_BCoM        = sym(eye(3));
body(i).geometry.type       = 'cylinder';
body(i).geometry.issolid    = true;
body(i).geometry.params     = [r_A l_A]; 
body(i).geometry.values     = [0.015 0.4];
body(i).geometry.offsets.r  = [0 0 -0.2].';
body(i).geometry.offsets.C  = eye(3);
body(i).geometry.color      = [0.6 0.6 0.6];

% Rotor Hub 2
i=6;
body(i)                     = RigidBodyDescription_v2;
body(i).name                = 'none';
body(i).ktree.nodeid        = i;
body(i).ktree.parents       = i-1;
body(i).cs.P_r_PB           = [0 0 l_A].';
body(i).cs.C_PB             = getRotationMatrixY(-pi/2);
body(i).param.m             = m_H;
body(i).param.B_Theta_B     = [];
body(i).param.B_r_BCoM      = [0 0 0].';
body(i).param.C_BCoM        = sym(eye(3));
body(i).geometry.type       = 'cylinder';
body(i).geometry.issolid    = true;
body(i).geometry.params     = [r_H l_H]; 
body(i).geometry.values     = [0.03 0.05];
body(i).geometry.offsets.r  = [0 0 0].';
body(i).geometry.offsets.C  = eye(3);
body(i).geometry.color      = [0.6 0.6 0.6];

% Rotor Disc 2
i=7;
body(i)                     = RigidBodyDescription_v2;
body(i).name                = 'none';
body(i).ktree.nodeid        = i;
body(i).ktree.parents       = i-1;
body(i).cs.P_r_PB           = [0 0 l_H/2].';
body(i).cs.C_PB             = sym(eye(3));
body(i).param.m             = m_R;
body(i).param.B_Theta_B     = [];
body(i).param.B_r_BCoM      = [0 0 0].';
body(i).param.C_BCoM        = eye(3);
body(i).geometry.type       = 'cylinder';
body(i).geometry.issolid    = true;
body(i).geometry.params     = [r_R l_R]; 
body(i).geometry.values     = [0.12 0.005];
body(i).geometry.offsets.r  = [0 0 0].';
body(i).geometry.offsets.C  = eye(3);
body(i).geometry.color      = [0.6 0.6 0.8];

% Rotor Arm 3
i=8;
body(i)                     = RigidBodyDescription_v2;
body(i).name                = 'none';
body(i).ktree.nodeid        = i;
body(i).ktree.parents       = 1;
body(i).cs.P_r_PB           = [-l_B/2 0 0].';
body(i).cs.C_PB             = getRotationMatrixZ(-pi)*getRotationMatrixY(pi/2);
body(i).param.m             = m_A;
body(i).param.B_Theta_B     = [];
body(i).param.B_r_BCoM      = [0 0 d_Az].'; 
body(i).param.C_BCoM        = sym(eye(3));
body(i).geometry.type       = 'cylinder';
body(i).geometry.issolid    = true;
body(i).geometry.params     = [r_A l_A]; 
body(i).geometry.values     = [0.015 0.4];
body(i).geometry.offsets.r  = [0 0 -0.2].';
body(i).geometry.offsets.C  = eye(3);
body(i).geometry.color      = [0.6 0.6 0.6];

% Rotor Hub 3
i=9;
body(i)                     = RigidBodyDescription_v2;
body(i).name                = 'none';
body(i).ktree.nodeid        = i;
body(i).ktree.parents       = i-1;
body(i).cs.P_r_PB           = [0 0 l_A].';
body(i).cs.C_PB             = getRotationMatrixY(-pi/2);
body(i).param.m             = m_H;
body(i).param.B_Theta_B     = [];
body(i).param.B_r_BCoM      = [0 0 0].';
body(i).param.C_BCoM        = sym(eye(3));
body(i).geometry.type       = 'cylinder';
body(i).geometry.issolid    = true;
body(i).geometry.params     = [r_H l_H]; 
body(i).geometry.values     = [0.03 0.05];
body(i).geometry.offsets.r  = [0 0 0].';
body(i).geometry.offsets.C  = eye(3);
body(i).geometry.color      = [0.6 0.6 0.6];

% Rotor Disc 3
i=10;
body(i)                     = RigidBodyDescription_v2;
body(i).name                = 'none';
body(i).ktree.nodeid        = i;
body(i).ktree.parents       = i-1;
body(i).cs.P_r_PB           = [0 0 l_H/2].';
body(i).cs.C_PB             = sym(eye(3));
body(i).param.m             = m_R;
body(i).param.B_Theta_B     = [];
body(i).param.B_r_BCoM      = [0 0 0].';
body(i).param.C_BCoM        = eye(3);
body(i).geometry.type       = 'cylinder';
body(i).geometry.issolid    = true;
body(i).geometry.params     = [r_R l_R]; 
body(i).geometry.values     = [0.12 0.005];
body(i).geometry.offsets.r  = [0 0 0].';
body(i).geometry.offsets.C  = eye(3);
body(i).geometry.color      = [0.6 0.6 0.8];

% Rotor Arm 4
i=11;
body(i)                     = RigidBodyDescription_v2;
body(i).name                = 'none';
body(i).ktree.nodeid        = i;
body(i).ktree.parents       = 1;
body(i).cs.P_r_PB           = [0 w_B/2 0].';
body(i).cs.C_PB             = getRotationMatrixZ(-pi*(3/2))*getRotationMatrixY(pi/2);
body(i).param.m             = m_A;
body(i).param.B_Theta_B     = [];
body(i).param.B_r_BCoM      = [0 0 d_Az].';
body(i).param.C_BCoM        = sym(eye(3));
body(i).geometry.type       = 'cylinder';
body(i).geometry.issolid    = true;
body(i).geometry.params     = [r_A l_A]; 
body(i).geometry.values     = [0.015 0.4];
body(i).geometry.offsets.r  = [0 0 -0.2].';
body(i).geometry.offsets.C  = eye(3);
body(i).geometry.color      = [0.6 0.6 0.6];

% Rotor Hub 4
i=12;
body(i)                     = RigidBodyDescription_v2;
body(i).name                = 'none';
body(i).ktree.nodeid        = i;
body(i).ktree.parents       = i-1;
body(i).cs.P_r_PB           = [0 0 l_A].';
body(i).cs.C_PB             = getRotationMatrixY(-pi/2);
body(i).param.m             = m_H;
body(i).param.B_Theta_B     = [];
body(i).param.B_r_BCoM      = [0 0 0].';
body(i).param.C_BCoM        = sym(eye(3));
body(i).geometry.type       = 'cylinder';
body(i).geometry.issolid    = true;
body(i).geometry.params     = [r_H l_H]; 
body(i).geometry.values     = [0.03 0.05];
body(i).geometry.offsets.r  = [0 0 0].';
body(i).geometry.offsets.C  = eye(3);
body(i).geometry.color      = [0.6 0.6 0.6];

% Rotor Disc 4
i=13;
body(i)                     = RigidBodyDescription_v2;
body(i).name                = 'none';
body(i).ktree.nodeid        = i;
body(i).ktree.parents       = i-1;
body(i).cs.P_r_PB           = [0 0 l_H/2].';
body(i).cs.C_PB             = sym(eye(3));
body(i).param.m             = m_R;
body(i).param.B_Theta_B     = [];
body(i).param.B_r_BCoM      = [0 0 0].';
body(i).param.C_BCoM        = eye(3);
body(i).geometry.type       = 'cylinder';
body(i).geometry.issolid    = true;
body(i).geometry.params     = [r_R l_R]; 
body(i).geometry.values     = [0.12 0.005];
body(i).geometry.offsets.r  = [0 0 0].';
body(i).geometry.offsets.C  = eye(3);
body(i).geometry.color      = [0.6 0.6 0.8];

%% Definition of External Forces & Torques

% Environmental wrenches
j=1;
ftel(j) = ForceTorqueDescription_v2;
ftel(j).name    = 'W_E';
ftel(j).type    = 'wrench';
ftel(j).body_P  = 0; 
ftel(j).body_B  = 1;
ftel(j).P_r_R   = [];
ftel(j).B_r_A   = sym([0 0 0].');
ftel(j).I_F     = [F_Ex F_Ey F_Ez].';
ftel(j).I_T     = [T_Ex T_Ey T_Ez].';

% Total rotor wrench
j=2;
ftel(j) = ForceTorqueDescription_v2;
ftel(j).name    = 'W_R1';
ftel(j).type    = 'wrench';
ftel(j).body_P  = 0; 
ftel(j).body_B  = 4;
ftel(j).P_r_R   = [];
ftel(j).B_r_A   = sym([0 0 0].');
ftel(j).B_F     = [0 0 F_T1].';
ftel(j).B_T     = [0 0 T_Q1].';

% Total rotor wrench
j=3;
ftel(j) = ForceTorqueDescription_v2;
ftel(j).name    = 'W_R2';
ftel(j).type    = 'wrench';
ftel(j).body_P  = 0; 
ftel(j).body_B  = 7;
ftel(j).P_r_R   = [];
ftel(j).B_r_A   = sym([0 0 0].');
ftel(j).B_F     = [0 0 F_T2].';
ftel(j).B_T     = [0 0 T_Q2].';

% Total rotor wrench
j=4;
ftel(j) = ForceTorqueDescription_v2;
ftel(j).name    = 'W_R3';
ftel(j).type    = 'wrench';
ftel(j).body_P  = 0; 
ftel(j).body_B  = 10;
ftel(j).P_r_R   = [];
ftel(j).B_r_A   = sym([0 0 0].');
ftel(j).B_F     = [0 0 F_T3].';
ftel(j).B_T     = [0 0 T_Q3].';

% Total rotor wrench
j=5;
ftel(j) = ForceTorqueDescription_v2;
ftel(j).name    = 'W_R4';
ftel(j).type    = 'wrench';
ftel(j).body_P  = 0; 
ftel(j).body_B  = 13;
ftel(j).P_r_R   = [];
ftel(j).B_r_A   = sym([0 0 0].');
ftel(j).B_F     = [0 0 F_T4].';
ftel(j).B_T     = [0 0 T_Q4].';

%% System Definition

% The system has no internal degrees of freedom
q_j  = [].';

% Controllable joint forces/torques
tau_j = [F_T1 F_T2 F_T3 F_T4 T_Q1 T_Q2 T_Q3 T_Q4].';

% External forces/torques
tau_env = [F_Ex F_Ey F_Ez T_Ex T_Ey T_Ez].';

%% Generate Full System Model using proNEu.v2

robot_name = 'QuadRotorModel';

% Generate the model object
robotmdl = RobotModel(body, ftel, q_j, tau_j, tau_env, I_a_g, 'name', robot_name, 'type', 'floating', 'orientation', 'cardanxyz', 'method', 'proneu', 'symsteps', 100);

%% Symbolic Simplifications (Advanced)

% Set number of symbolic simplification steps
ns = 100;

% Test symbolic simplifciations of M,b,g - use serial computations
robotmdl.dynamics.symbols.simplifydynamics('elementwise', ns);

%% Generate Numerical Computations

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
