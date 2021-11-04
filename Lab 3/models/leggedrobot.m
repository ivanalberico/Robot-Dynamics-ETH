% Optional calls for workspace resetting/cleaning
close all; clear all; clear classes; clc;

%% World Gravity

% Gravitational potential field in inertial world-frame
syms grav real;
e_I_g = [0 0 1].';
I_a_g = grav*e_I_g;

%% System Parameters & Variables

% Body masses
syms m_B m_HT m_HS m_FT m_FS m_UA m_LA m_HA real;

% Body geometry parameters
syms l_B w_B h_B real;
syms r_HT l_HT real;
syms r_HS l_HS real;
syms r_HF real;
syms r_FT l_FT real;
syms r_FS l_FS real;
syms r_FF real;
syms r_UA l_UA real;
syms r_LA l_LA real;
syms r_HA l_HA real;
syms r_EE real;

% Declare all system variables
% State variables
syms x_B z_B theta_B phi_RHFE phi_RKFE phi_FHFE phi_FKFE phi_SH phi_EL phi_WR real;
% Applicable joint torques
syms tau_RHFE tau_RKFE tau_FHFE tau_FKFE tau_SH tau_EL tau_WR real;
% Feet and End-Effector contact forces and torques (6D wrenches)
syms f_HFx f_HFy f_HFz t_HFx t_HFy t_HFz 
syms f_FFx f_FFy f_FFz t_FFx t_FFy t_FFz 
syms f_EEx f_EEy f_EEz t_EEx t_EEy t_EEz

%% Multi-Body System Description (Kinematic Tree)

% Base
i=1;
body(i)                     = RigidBodyDescription_v2;
body(i).name                = 'B';
body(i).ktree.nodeid        = 1;
body(i).ktree.parents       = 0;
body(i).cs.P_r_PB           = [x_B 0 z_B].';
body(i).cs.C_PB             = getRotationMatrixY(theta_B);
body(i).param.m             = m_B;
body(i).param.B_Theta       = [];
body(i).param.B_r_BCoM      = [0 0 0].'; 
body(i).param.C_BCoM        = sym(eye(3));
body(i).geometry.type       = 'cuboid';
body(i).geometry.issolid    = true;
body(i).geometry.params     = [l_B w_B h_B] ; 
body(i).geometry.values     = [0.6 0.2 0.1];
body(i).geometry.offsets.r  = [0 0 0].';
body(i).geometry.offsets.C  = eye(3);
body(i).geometry.color      = [197 172 202]/255;

% Hind Thigh (HT)
i=2;
body(i)                     = RigidBodyDescription_v2;
body(i).name                = 'HT';
body(i).ktree.nodeid        = 2;
body(i).ktree.parents       = 1;
body(i).cs.P_r_PB           = [-l_B/2 0 -h_B/2].';
body(i).cs.C_PB             = getRotationMatrixY(phi_RHFE);
body(i).param.m             = m_HT;
body(i).param.B_Theta       = [];
body(i).param.B_r_BCoM      = [0 0 -l_HT/2].'; 
body(i).param.C_BCoM        = sym(eye(3));
body(i).geometry.type       = 'cylinder';
body(i).geometry.issolid    = true;
body(i).geometry.params     = [r_HT l_HT]; 
body(i).geometry.values     = [0.015 0.3];
body(i).geometry.offsets.r  = [0 0 0.15].';
body(i).geometry.offsets.C  = eye(3);
body(i).geometry.color      = [0.6 0.6 0.6];

% Hind Shank (HS)
i=3;
body(i)                     = RigidBodyDescription_v2;
body(i).name                = 'HS';
body(i).ktree.nodeid        = 3;
body(i).ktree.parents       = 2;
body(i).cs.P_r_PB           = [0 0 -l_HT].';
body(i).cs.C_PB             = getRotationMatrixY(phi_RKFE);
body(i).param.m             = m_HS;
body(i).param.B_Theta       = [];
body(i).param.B_r_BCoM      = [0 0 -l_HT/2].'; 
body(i).param.C_BCoM        = sym(eye(3));
body(i).geometry.type       = 'cylinder';
body(i).geometry.issolid    = true;
body(i).geometry.params     = [r_HT l_HT]; 
body(i).geometry.values     = [0.015 0.3];
body(i).geometry.offsets.r  = [0 0 0.15].';
body(i).geometry.offsets.C  = eye(3);
body(i).geometry.color      = [0.6 0.6 0.6];

% Hind Foot (HF)
i=4;
body(i) = RigidBodyDescription_v2;
body(i).name = 'HF';
body(i).ktree.nodeid = 4;
body(i).ktree.parents = 3;
body(i).cs.P_r_PB = [0 0 -l_HS].';
body(i).cs.C_PB = sym(eye(3));
body(i).geometry.type = 'sphere';
body(i).geometry.issolid = false;
body(i).geometry.params = [r_HF]; 
body(i).geometry.values = [0.02];
body(i).geometry.offsets.r = [0 0 0].';
body(i).geometry.offsets.C = eye(3);
body(i).geometry.color = [0.8 0.6 0.6];

% Front Thigh (FT)
i=5;
body(i)                     = RigidBodyDescription_v2;
body(i).name                = 'FT';
body(i).ktree.nodeid        = 5;
body(i).ktree.parents       = 1;
body(i).cs.P_r_PB           = [l_B/2 0 -h_B/2].';
body(i).cs.C_PB             = getRotationMatrixY(phi_FHFE);
body(i).param.m             = m_FT;
body(i).param.B_Theta       = [];
body(i).param.B_r_BCoM      = [0 0 -l_FT/2].'; 
body(i).param.C_BCoM        = sym(eye(3));
body(i).geometry.type       = 'cylinder';
body(i).geometry.issolid    = true;
body(i).geometry.params     = [r_FT l_FT]; 
body(i).geometry.values     = [0.015 0.3];
body(i).geometry.offsets.r  = [0 0 0.15].';
body(i).geometry.offsets.C  = eye(3);
body(i).geometry.color      = [0.6 0.6 0.6];

% Front Shank (FS)
i=6;
body(i)                     = RigidBodyDescription_v2;
body(i).name                = 'FS';
body(i).ktree.nodeid        = 6;
body(i).ktree.parents       = 5;
body(i).cs.P_r_PB           = [0 0 -l_FT].';
body(i).cs.C_PB             = getRotationMatrixY(phi_FKFE);
body(i).param.m             = m_FS;
body(i).param.B_Theta       = [];
body(i).param.B_r_BCoM      = [0 0 -l_FT/2].'; 
body(i).param.C_BCoM        = sym(eye(3));
body(i).geometry.type       = 'cylinder';
body(i).geometry.issolid    = true;
body(i).geometry.params     = [r_FT l_FT]; 
body(i).geometry.values     = [0.015 0.3];
body(i).geometry.offsets.r  = [0 0 0.15].';
body(i).geometry.offsets.C  = eye(3);
body(i).geometry.color      = [0.6 0.6 0.6];

% Front Foot (FF)
i=7;
body(i) = RigidBodyDescription_v2;
body(i).name = 'FF';
body(i).ktree.nodeid = 7;
body(i).ktree.parents = 6;
body(i).cs.P_r_PB = [0 0 -l_FS].';
body(i).cs.C_PB = sym(eye(3));
body(i).geometry.type = 'sphere';
body(i).geometry.issolid = false;
body(i).geometry.params = [r_FF]; 
body(i).geometry.values = [0.02];
body(i).geometry.offsets.r = [0 0 0].';
body(i).geometry.offsets.C = eye(3);
body(i).geometry.color = [0.8 0.6 0.6];

% Upper Arm (UA)
i=8;
body(i)                     = RigidBodyDescription_v2;
body(i).name                = 'UA';
body(i).ktree.nodeid        = 8;
body(i).ktree.parents       = 1;
body(i).cs.P_r_PB           = [l_B/4 0 h_B/2].';
body(i).cs.C_PB             = getRotationMatrixY(phi_SH);
body(i).param.m             = m_UA;
body(i).param.B_Theta       = [];
body(i).param.B_r_BCoM      = [0 0 l_UA/2].'; 
body(i).param.C_BCoM        = sym(eye(3));
body(i).geometry.type       = 'cylinder';
body(i).geometry.issolid    = true;
body(i).geometry.params     = [r_UA l_UA]; 
body(i).geometry.values     = [0.015 0.3];
body(i).geometry.offsets.r  = [0 0 -0.15].';
body(i).geometry.offsets.C  = eye(3);
body(i).geometry.color      = [0.6 0.6 0.6];

% Lower Arm (LA)
i=9;
body(i)                     = RigidBodyDescription_v2;
body(i).name                = 'LA';
body(i).ktree.nodeid        = 9;
body(i).ktree.parents       = 8;
body(i).cs.P_r_PB           = [0 0 l_UA].';
body(i).cs.C_PB             = getRotationMatrixY(phi_EL);
body(i).param.m             = m_LA;
body(i).param.B_Theta       = [];
body(i).param.B_r_BCoM      = [0 0 l_LA/2].'; 
body(i).param.C_BCoM        = sym(eye(3));
body(i).geometry.type       = 'cylinder';
body(i).geometry.issolid    = true;
body(i).geometry.params     = [r_LA l_LA]; 
body(i).geometry.values     = [0.015 0.3];
body(i).geometry.offsets.r  = [0 0 -0.15].';
body(i).geometry.offsets.C  = eye(3);
body(i).geometry.color      = [0.6 0.6 0.6];

% Arm Hand (HA)
i=10;
body(i)                     = RigidBodyDescription_v2;
body(i).name                = 'HA';
body(i).ktree.nodeid        = 10;
body(i).ktree.parents       = 9;
body(i).cs.P_r_PB           = [0 0 l_LA].';
body(i).cs.C_PB             = getRotationMatrixY(phi_WR);
body(i).param.m             = m_HA;
body(i).param.B_Theta       = [];
body(i).param.B_r_BCoM      = [0 0 l_HA/2].'; 
body(i).param.C_BCoM        = sym(eye(3));
body(i).geometry.type       = 'cylinder';
body(i).geometry.issolid    = true;
body(i).geometry.params     = [r_HA l_HA]; 
body(i).geometry.values     = [0.015 0.15];
body(i).geometry.offsets.r  = [0 0 -0.075].';
body(i).geometry.offsets.C  = eye(3);
body(i).geometry.color      = [0.6 0.6 0.6];


% Arm End-Effector (EE)
i=11;
body(i) = RigidBodyDescription_v2;
body(i).name = 'EE';
body(i).ktree.nodeid = 11;
body(i).ktree.parents = 10;
body(i).cs.P_r_PB = [0 0 l_HA].';
body(i).cs.C_PB = sym(eye(3));
body(i).geometry.type = 'sphere';
body(i).geometry.issolid = false;
body(i).geometry.params = [r_EE]; 
body(i).geometry.values = [0.02];
body(i).geometry.offsets.r = [0 0 0].';
body(i).geometry.offsets.C = eye(3);
body(i).geometry.color = [0.8 0.6 0.6];


%% Definition of External Forces & Torques

%
% Joint Torques
%

j=1;
ftel(j) = ForceTorqueDescription_v2;
ftel(j).name    = 'tau_RHFE';
ftel(j).type    = 'rotational';
ftel(j).body_P  = 1; 
ftel(j).body_B  = 2;
ftel(j).B_T     = [0 tau_RHFE 0].';

j=2;
ftel(j) = ForceTorqueDescription_v2;
ftel(j).name    = 'tau_RKFE';
ftel(j).type    = 'rotational';
ftel(j).body_P  = 2; 
ftel(j).body_B  = 3;
ftel(j).B_T     = [0 tau_RKFE 0].';

j=3;
ftel(j) = ForceTorqueDescription_v2;
ftel(j).name    = 'tau_FHFE';
ftel(j).type    = 'rotational';
ftel(j).body_P  = 1; 
ftel(j).body_B  = 5;
ftel(j).B_T     = [0 tau_FHFE 0].';

j=4;
ftel(j) = ForceTorqueDescription_v2;
ftel(j).name    = 'tau_FKFE';
ftel(j).type    = 'rotational';
ftel(j).body_P  = 5; 
ftel(j).body_B  = 6;
ftel(j).B_T     = [0 tau_FKFE 0].';

j=5;
ftel(j) = ForceTorqueDescription_v2;
ftel(j).name    = 'tau_SH';
ftel(j).type    = 'rotational';
ftel(j).body_P  = 1; 
ftel(j).body_B  = 8;
ftel(j).B_T     = [0 tau_SH 0].';

j=6;
ftel(j) = ForceTorqueDescription_v2;
ftel(j).name    = 'tau_EL';
ftel(j).type    = 'rotational';
ftel(j).body_P  = 8; 
ftel(j).body_B  = 9;
ftel(j).B_T     = [0 tau_EL 0].';

j=7;
ftel(j) = ForceTorqueDescription_v2;
ftel(j).name    = 'tau_WR';
ftel(j).type    = 'rotational';
ftel(j).body_P  = 9; 
ftel(j).body_B  = 10;
ftel(j).B_T     = [0 tau_WR 0].';

%
% Contact Wrenches
%

j=8;
ftel(j) = ForceTorqueDescription_v2;
ftel(j).name    = 'w_HF';
ftel(j).type    = 'wrench';
ftel(j).body_P  = 0; 
ftel(j).body_B  = 4;
ftel(j).P_r_R   = sym([0 0 0].');
ftel(j).B_r_A   = sym([0 0 -r_HF].');
ftel(j).I_F     = [f_HFx f_HFy f_HFz].';
ftel(j).I_T     = [t_HFx t_HFy t_HFz].';

j=9;
ftel(j) = ForceTorqueDescription_v2;
ftel(j).name    = 'w_FF';
ftel(j).type    = 'wrench';
ftel(j).body_P  = 0; 
ftel(j).body_B  = 7;
ftel(j).P_r_R   = sym([0 0 0].');
ftel(j).B_r_A   = sym([0 0 -r_FF].');
ftel(j).I_F     = [f_FFx f_FFy f_FFz].';
ftel(j).I_T     = [t_FFx t_FFy t_FFz].';

j=10;
ftel(j) = ForceTorqueDescription_v2;
ftel(j).name    = 'w_EE';
ftel(j).type    = 'wrench';
ftel(j).body_P  = 0; 
ftel(j).body_B  = 11;
ftel(j).P_r_R   = sym([0 0 0].');
ftel(j).B_r_A   = sym([0 0 0].');
ftel(j).I_F     = [f_EEx f_EEy f_EEz].';
ftel(j).I_T     = [t_EEx t_EEy t_EEz].';


%% System Definitions

% Definition of the joint DoFs of 2-link system
q_j  = [x_B z_B theta_B phi_RHFE phi_RKFE phi_FHFE phi_FKFE phi_SH phi_EL phi_WR].';

% Controllable joint forces/torques
tau_j = [tau_RHFE tau_RKFE tau_FHFE tau_FKFE tau_SH tau_EL tau_WR].';

% External forces/torques
tau_env = [f_HFx f_HFy f_HFz t_HFx t_HFy t_HFz f_FFx f_FFy f_FFz t_FFx t_FFy t_FFz f_EEx f_EEy f_EEz t_EEx t_EEy t_EEz].';

%% Generate Full System Model using proNEu.v2

% Give a name to the model
robot_name = 'LeggedRobotModel';

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
[fpath,fname,fext] = fileparts(mfilename('fullpath'));
dpath = strrep(fpath, 'generators', 'models/');

% Store generated model in the appropriate directory
save(strcat(dpath,robotmdl.name), 'robotmdl');

%% 
% EOF
