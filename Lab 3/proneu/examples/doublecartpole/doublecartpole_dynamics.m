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
%   File:           doublecartpole_dynamics.m
%
%   Authors:        Vassilios Tsounis, tsounisv@ethz.ch
%
%   Date:           28/9/2016
%
%   Desciption:     Analytical derivation of the EoM for a Cart-Pole
%                   system.
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
syms m_C m_P m_Q real;

syms d_Cx d_Cy d_Cz real;
syms l_C w_C h_C real;

syms d_Px d_Py d_Pz real;
syms r_P l_P real;

syms d_Qx d_Qy d_Qz real;
syms r_Q l_Q real;

% Declare all system variables
syms x_C phi_P phi_Q real;
syms F_C F_D real;

%% Multi-Body System Description

% Cart
i=1;
body(i)                     = RigidBodyDescription_v2;
body(i).name                = 'C';
body(i).ktree.nodeid        = i;
body(i).ktree.parents       = i-1;
body(i).cs.P_r_PB           = [x_C 0 0].';
body(i).cs.C_PB             = sym(eye(3));
body(i).param.m             = m_C;
body(i).param.B_Theta       = [];
body(i).param.B_r_BCoM      = [d_Cx d_Cy d_Cz].'; 
body(i).param.C_BCoM        = sym(eye(3));
body(i).geometry.type       = 'cuboid';
body(i).geometry.issolid    = true;
body(i).geometry.params     = [l_C w_C h_C] ; 
body(i).geometry.values     = [0.3 0.3 0.2];
body(i).geometry.offsets.r  = [0 0 -0.1].';
body(i).geometry.offsets.C  = eye(3);
body(i).geometry.color      = [197 172 202]/255;

% First pole
i=2;
body(i)                     = RigidBodyDescription_v2;
body(i).name                = 'P';
body(i).ktree.nodeid        = i;
body(i).ktree.parents       = i-1;
body(i).cs.P_r_PB           = [0 0 0].';
body(i).cs.C_PB             = getRotationMatrixY(phi_P);
body(i).param.m             = m_P;
body(i).param.B_Theta       = [];
body(i).param.B_r_BCoM      = [d_Px d_Py d_Pz].'; 
body(i).param.C_BCoM        = sym(eye(3));
body(i).geometry.type       = 'cylinder';
body(i).geometry.issolid    = true;
body(i).geometry.params     = [r_P l_P]; 
body(i).geometry.values     = [0.05 1.0];
body(i).geometry.offsets.r  = [0 0 -0.5].';
body(i).geometry.offsets.C  = eye(3);
body(i).geometry.color      = [0.6 0.6 0.6];

% Second pole
i=3;
body(i)                     = RigidBodyDescription_v2;
body(i).name                = 'Q';
body(i).ktree.nodeid        = i;
body(i).ktree.parents       = i-1;
body(i).cs.P_r_PB           = [0 0 l_P].';
body(i).cs.C_PB             = getRotationMatrixY(phi_Q);
body(i).param.m             = m_Q;
body(i).param.B_Theta       = [];
body(i).param.B_r_BCoM      = [d_Qx d_Qy d_Qz].'; 
body(i).param.C_BCoM        = sym(eye(3));
body(i).geometry.type       = 'cylinder';
body(i).geometry.issolid    = true;
body(i).geometry.params     = [r_Q l_Q]; 
body(i).geometry.values     = [0.05 1.0];
body(i).geometry.offsets.r  = [0 0 -0.5].';
body(i).geometry.offsets.C  = eye(3);
body(i).geometry.color      = [0.6 0.6 0.6];

%% Definition of External Forces & Torques

j=1;
ftel(j) = ForceTorqueDescription_v2;
ftel(j).name    = 'F_cart';
ftel(j).type    = 'linear';
ftel(j).body_P  = 0; 
ftel(j).body_B  = 1;
ftel(j).P_r_R   = sym([0 0 0].');
ftel(j).B_r_A   = sym([0 0 0].');
ftel(j).I_F     = [F_C 0 0].';

j=2;
ftel(j) = ForceTorqueDescription_v2;
ftel(j).name    = 'F_dist';
ftel(j).type    = 'linear';
ftel(j).body_P  = 0; 
ftel(j).body_B  = 3;
ftel(j).P_r_R   = sym([0 0 0].');
ftel(j).B_r_A   = sym([0 0 0].');
ftel(j).I_F     = [F_D 0 0].';

%% System Definitions

% Definition of the joint DoFs of 2-link system
q_j  = [x_C phi_P phi_Q].';

% Controllable joint forces/torques
tau_j = [F_C].';

% External forces/torques
tau_env = [F_D].';

%% Generate Full System Model using proNEu.v2

% Give a name to the model
robot_name = 'DoubleCartPoleModel';

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
