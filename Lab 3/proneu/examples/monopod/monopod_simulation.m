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
%   File:           monopod_simulation.m
%
%   Authors:        Vassilios Tsounis, tsounisv@ethz.ch
%
%   Date:           10/12/2016
%
%   Desciption:     Simulation of a planar cart-pole system.
%

% Optional calls for workspace resetting/cleaning
close all; clear all; clear classes; clc;

%% Load the Model Data

load('MonopodModel.mat');
load('MonopodWorld.mat');

%% Create a Controller

% Generate a robot controller using an external implementation
% controller = RobotController(@monopod_jointspace_pid_controller);
% controller = RobotController(@monopod_jointspace_impedance_controller);
controller = RobotController(@monopod_jointspace_inverse_dynamics_controller);

%% Generate the Simulation Environment

% Create the robot simulation engine
robotsim = RobotSimulator(robotmdl, controller, worldmdl, 'solver', 'fixedstep');

% Data Plots
% robotsim.scope.open();

%% Set the Model Parameters

% System parameter values
% params = [d_Bx, d_By, d_Bz, d_Sx, d_Sy, d_Sz, d_Tx, d_Ty, d_Tz, grav, h_B, l_B, l_S, l_T, m_B, m_S, m_T, r_F, r_S, r_T, w_B]
mparams  = [0.0   0.0   0.0   0.0   0.0   -0.5  0.0   0.0   -0.5  9.81  0.2  0.3  1.0  1.0  3.0  0.2  0.2  0.07 0.05 0.05 0.3].';

% Store the parameters
robotmdl.parameters.values = mparams;

%% Create the Visualizations

% Configurations
fontsize = 10;
csfscale = 0.1;
posoffset = [0; 0; 0.5];
zoom = 1.0;

% Generate 3D Visualization instance
robotviz = RobotVisualization(robotmdl,worldmdl,fontsize,csfscale,zoom,posoffset);
robotviz.open();
robotviz.load();
% set(gcf, 'Position', get(0, 'Screensize'));

% Give the simulator access to the 3D visualization
robotsim.setvisualizer(robotviz);

%% Run Simulation

% Set the system's initial conditions
xinit = [2.0 -pi/4 pi/2 0.0 0.0 0.0].';

% Set the system parameters used by the simulation
robotsim.setup('xinit', xinit);

% Simulation time configurations
simconf.tstart  = 0;
simconf.tstop   = 5.0;
simconf.tstep   = 1e-4;
simconf.fps     = 20.0;

% Execute the simlation engine
robotsim.run(simconf);

%%
% EOF
