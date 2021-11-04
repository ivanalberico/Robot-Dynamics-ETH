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
%   File:           monopod_dynamics.m
%
%   Authors:        Vassilios Tsounis, tsounisv@ethz.ch
%
%   Date:           27/11/2016
%
%   Desciption:     Analytical derivation of the EoM for a 2D monopod on
%                   a guided vertical rail.
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
syms m_B m_T m_S real;

syms d_Bx d_By d_Bz real;
syms l_B w_B h_B real;

syms d_Tx d_Ty d_Tz real;
syms r_T l_T real;

syms d_Sx d_Sy d_Sz real;
syms r_S l_S real;

syms r_F real;

% Declare all system variables
syms z_B dz_B real;
syms phi_H phi_K real;
syms T_H T_K real;
syms F_Cx F_Cy F_Cz T_Cx T_Cy T_Cz real;

%% Multi-Body System Description

i=1;
body(i) = RigidBodyDescription_v2;
body(i).name = 'B';
body(i).ktree.nodeid = i;
body(i).ktree.parents = i-1;
body(i).cs.P_r_PB = [0 0 z_B].';
body(i).cs.C_PB = sym(eye(3));
body(i).param.m = m_B;
body(i).param.B_Theta = [];
body(i).param.B_r_BCoM = [d_Bx d_By d_Bz].'; 
body(i).param.C_BCoM = sym(eye(3));
body(i).geometry.type = 'cuboid';
body(i).geometry.issolid = false;
body(i).geometry.params = [l_B w_B h_B] ; 
body(i).geometry.values = [0.3 0.3 0.2];
body(i).geometry.offsets.r = [0 0 0].';
body(i).geometry.offsets.C = eye(3);
body(i).geometry.color = [197 172 202]/255;

i=2;
body(i) = RigidBodyDescription_v2;
body(i).name = 'T';
body(i).ktree.nodeid = i;
body(i).ktree.parents = i-1;
body(i).cs.P_r_PB = [0 0 0].';
body(i).cs.C_PB = getRotationMatrixY(phi_H);
body(i).param.m = m_T;
body(i).param.B_Theta = [];
body(i).param.B_r_BCoM = [d_Tx d_Ty d_Tz]; 
body(i).param.C_BCoM = sym(eye(3));
body(i).geometry.type = 'cylinder';
body(i).geometry.issolid = false;
body(i).geometry.params = [r_T l_T]; 
body(i).geometry.values = [0.05 1.0];
body(i).geometry.offsets.r = [0 0 0.5].';
body(i).geometry.offsets.C = eye(3);
body(i).geometry.color = [0.6 0.6 0.6];

i=3;
body(i) = RigidBodyDescription_v2;
body(i).name = 'S';
body(i).ktree.nodeid = i;
body(i).ktree.parents = i-1;
body(i).cs.P_r_PB = [0 0 -l_T].';
body(i).cs.C_PB = getRotationMatrixY(phi_K);
body(i).param.m = m_S;
body(i).param.B_Theta = [];
body(i).param.B_r_BCoM = [d_Sx d_Sy d_Sz]; 
body(i).param.C_BCoM = sym(eye(3));
body(i).geometry.type = 'cylinder';
body(i).geometry.issolid = false;
body(i).geometry.params = [r_S l_S]; 
body(i).geometry.values = [0.05 1.0];
body(i).geometry.offsets.r = [0 0 0.5].';
body(i).geometry.offsets.C = eye(3);
body(i).geometry.color = [0.6 0.6 0.6];

i=4;
body(i) = RigidBodyDescription_v2;
body(i).name = 'F';
body(i).ktree.nodeid = i;
body(i).ktree.parents = i-1;
body(i).cs.P_r_PB = [0 0 -l_S].';
body(i).cs.C_PB = sym(eye(3));
body(i).geometry.type = 'sphere';
body(i).geometry.issolid = false;
body(i).geometry.params = [r_F]; 
body(i).geometry.values = [0.07];
body(i).geometry.offsets.r = [0 0 0].';
body(i).geometry.offsets.C = eye(3);
body(i).geometry.color = [0.8 0.6 0.6];

%% Definition of External Forces & Torques

j=1;
ftel(j) = ForceTorqueDescription_v2;
ftel(j).name    = 'T_HFE';
ftel(j).type    = 'rotational';
ftel(j).body_P  = 1; 
ftel(j).body_B  = 2;
ftel(j).B_T     = [0 T_H 0].';

j=2;
ftel(j) = ForceTorqueDescription_v2;
ftel(j).name    = 'T_KFE';
ftel(j).type    = 'rotational';
ftel(j).body_P  = 2; 
ftel(j).body_B  = 3;
ftel(j).B_T     = [0 T_K 0].';

j=3;
ftel(j) = ForceTorqueDescription_v2;
ftel(j).name    = 'W_Foot';
ftel(j).type    = 'wrench';
ftel(j).body_P  = 0; 
ftel(j).body_B  = 4;
ftel(j).P_r_R   = sym([0 0 0].');
ftel(j).B_r_A   = sym([0 0 -r_F].');
ftel(j).I_F     = [F_Cx F_Cy F_Cz].';
ftel(j).I_T     = [T_Cx T_Cy T_Cz].';

%% System Definitions

% Definition of the joint DoFs of 2-link system
q_j  = [z_B phi_H phi_K].';

% Controllable joint forces/torques
tau_j = [T_H T_K].';

% External forces and torques
tau_env = [F_Cx F_Cy F_Cz T_Cx T_Cy T_Cz].';

%% Generate Full System Model using proNEu.v2

% Give a name to the model
robot_name = 'MonopodModel';

% Generate the model object
robotmdl = RobotModel(body, ftel, q_j, tau_j, tau_env, I_a_g, 'name', robot_name, 'type', 'fixed', 'method', 'proneu', 'symsteps', 100);

%% Symbolic Simplifications (Advanced)

% Set number of symbolic simplification steps
ns = 100;

% Test symbolic simplifciations of M,b,g - use serial computations
robotmdl.dynamics.symbols.simplifydynamics('elementwise', ns);

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
