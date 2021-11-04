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
%   File:           slip_dynamics.m
%
%   Authors:        Vassilios Tsounis, tsounisv@ethz.ch
%
%   Date:           26/04/2017
%
%   Desciption:     Dynamical model generation of a Spring-Loaded Inverted 
%                   Pendulum (SLIP).
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
syms m_B real;
syms r_B real;
syms d_Bx d_By d_Bz real;

syms m_Sb real;
syms r_Sb l_Sb real;
syms d_Sbx d_Sby d_Sbz real;

syms m_Sf real;
syms r_Sf l_Sf real;
syms d_Sfx d_Sfy d_Sfz real;

syms m_F real;
syms r_F real;
syms d_Fx d_Fy d_Fz real;

% Declare all system variables
syms zeta_S real;
syms T_Bx T_By real;
syms F_Sbf real;
syms F_Cx F_Cy F_Cz T_Cx T_Cy T_Cz real;

%% Multi-Body System Description

i=1;
body(i)                     = RigidBodyDescription_v2;
body(i).name                = 'B';
body(i).ktree.nodeid        = i;
body(i).ktree.parents       = i-1;
body(i).cs.P_r_PB           = [];
body(i).cs.C_PB             = [];
body(i).param.m             = m_B;
body(i).param.B_Theta       = [];
body(i).param.B_r_BCoM      = sym([0; 0; 0]); 
body(i).param.C_BCoM        = sym(eye(3));
body(i).geometry.type       = 'sphere';
body(i).geometry.issolid    = false;
body(i).geometry.params     = [r_B]; 
body(i).geometry.values     = [0.15];
body(i).geometry.offsets.r  = [0; 0; 0];
body(i).geometry.offsets.C  = eye(3);
body(i).geometry.color      = [197 172 202]/255;

i=2;
body(i)                     = RigidBodyDescription_v2;
body(i).name                = 'Sb';
body(i).ktree.nodeid        = i;
body(i).ktree.parents       = i-1;
body(i).cs.P_r_PB           = [0; 0; -l_Sb/2];
body(i).cs.C_PB             = sym(eye(3));
body(i).geometry.type       = 'cylinder';
body(i).geometry.issolid    = false;
body(i).geometry.params     = [r_Sb l_Sb]; 
body(i).geometry.values     = [0.03 0.5].';
body(i).geometry.offsets.r  = [0; 0; 0];
body(i).geometry.offsets.C  = eye(3);
body(i).geometry.color      = [0.6 0.6 0.6];

i=3;
body(i)                     = RigidBodyDescription_v2;
body(i).name                = 'Sf';
body(i).ktree.nodeid        = i;
body(i).ktree.parents       = i-1;
body(i).cs.P_r_PB           = [0; 0; -zeta_S];
body(i).cs.C_PB             = sym(eye(3));
body(i).geometry.type       = 'cylinder';
body(i).geometry.issolid    = false;
body(i).geometry.params     = [r_Sf l_Sf]; 
body(i).geometry.values     = [0.03 0.5].';
body(i).geometry.offsets.r  = [0; 0; 0];
body(i).geometry.offsets.C  = eye(3);
body(i).geometry.color      = [0.6 0.6 0.6];

i=4;
body(i)                     = RigidBodyDescription_v2;
body(i).name                = 'F';
body(i).ktree.nodeid        = i;
body(i).ktree.parents       = i-1;
body(i).cs.P_r_PB           = [0 0 -l_Sf/2].';
body(i).cs.C_PB             = sym(eye(3));
body(i).param.B_r_BCoM      = sym([0; 0; 0]); 
body(i).param.C_BCoM        = sym(eye(3));
body(i).param.m             = m_F;
body(i).param.B_Theta       = [];
body(i).geometry.type       = 'sphere';
body(i).geometry.issolid    = false;
body(i).geometry.params     = [r_F]; 
body(i).geometry.values     = [0.05];
body(i).geometry.offsets.r  = [0; 0; 0];
body(i).geometry.offsets.C  = eye(3);
body(i).geometry.color      = [0.8 0.6 0.6];


% C_IC = mapQuaternionToRotationMatrix(-q_B);
% F_r_FC = TODO;
% i=5;
% body(i)                     = RigidBodyDescription_v2;
% body(i).name                = 'C';
% body(i).ktree.nodeid        = i;
% body(i).ktree.parents       = i-1;
% body(i).cs.P_r_PB           = F_r_FC;
% body(i).cs.C_PB             = C_IC;
% body(i).geometry.type       = 'none';

%% Definition of External Forces & Torques

j=1;
ftel(j) = ForceTorqueDescription_v2;
ftel(j).name    = 'T_base';
ftel(j).type    = 'rotational';
ftel(j).body_P  = 0; 
ftel(j).body_B  = 1;
ftel(j).I_T     = [T_Bx T_By 0].';

j=2;
ftel(j) = ForceTorqueDescription_v2;
ftel(j).name    = 'F_Sbf';
ftel(j).type    = 'linear';
ftel(j).body_P  = 2; 
ftel(j).body_B  = 3;
ftel(j).P_r_R   = sym([0 0 0].');
ftel(j).B_r_A   = sym([0 0 0].');
ftel(j).B_F     = [0 0 F_Sbf].';

j=3;
ftel(j) = ForceTorqueDescription_v2;
ftel(j).name    = 'W_Foot';
ftel(j).type    = 'wrench';
ftel(j).body_P  = 0; 
ftel(j).body_B  = 4;
ftel(j).P_r_R   = sym([0 0 0].');
ftel(j).B_r_A   = sym([0 0 0].');
ftel(j).I_F     = [F_Cx F_Cy F_Cz].';
ftel(j).I_T     = [T_Cx T_Cy T_Cz].';

%% System Definitions

% Definition of the joint DoFs of 2-link system
q_j  = [zeta_S].';

% Controllable joint forces/torques
tau_j = [F_Sbf T_Bx T_By].';

% External forces and torques
tau_env = [F_Cx F_Cy F_Cz T_Cx T_Cy T_Cz].';

%% Generate Full System Model using proNEu.v2

% Give a name to the model
robot_name = 'SlipModel';

% Generate the model object
robotmdl = RobotModel(body, ...
                      ftel, ...
                      q_j, ...
                      tau_j, ...
                      tau_env, ...
                      I_a_g, ...
                      'name', robot_name, ...
                      'type', 'floating', ...
                      'orientation', 'quaternion', ...
                      'method', 'proneu', ...
                      'symsteps', 100);

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
