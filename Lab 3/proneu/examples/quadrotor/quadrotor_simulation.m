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
%   File:           quadrotor_simulation.m
%
%   Authors:        Vassilios Tsounis, tsounisv@ethz.ch
%
%   Date:           10/12/2016
%
%   Desciption:     Simulation of a quadrotor.
%

% Optional calls for workspace resetting/cleaning
close all; clear all; clear classes; clc;

%% Load the Model Data

load('QuadRotorModel.mat');
load('QuadRotorWorld.mat');

%% Set the Numerical Parameters

% Initial system state
% x   = [x_B,  y_B,   z_B,  roll_B, pitch_B, yaw_B,  dx_B, dy_B, dz_B, omega_Bx, omega_By, omega_Bz]
xinit = [-1    -1     0.0   0       0        pi/6    0     0     0     0         0         0       ].';

% System parameter values
% params = [d_Az, grav, h_B, l_A, l_B,  l_H,  l_R,   m_A,  m_B,  m_H, m_R,  r_A,   r_H,  r_R,  w_B]
params   = [0.2   9.81  0.08 0.4  0.15  0.00  0.005  0.05  1.5   0.1  0.01  0.015  0.03  0.12  0.15].';

% Set the model parameters
robotmdl.parameters.values = params;

%% Actuator Controller

% Define the internal actuator forces callback function.
controller = RobotController(@quadrotor_controller);

%% Generate the Simulation Environment

% Create the robot simulation engine
robotsim = RobotSimulator(robotmdl, controller, worldmdl, 'solver', 'fsfb');

%% Create the Visualizations

% Configurations
fontsize = 14;
csfscale = 0.2;
posoffset = [0; 0; 0.0];
zoom = 1.0;

% Generate 3D Visualization instance
robotviz = RobotVisualization(robotmdl,worldmdl,fontsize,csfscale,zoom,posoffset);
robotviz.open();
robotviz.load();
% set(gcf, 'Position', get(0, 'Screensize'));

% Give the simulator access to the 3D visualization
robotsim.setvisualizer(robotviz);

%% Run Simulation
clc;

% Set the system parameters used by the simulation
robotsim.setup('xinit', xinit);

% Set simulation time parameters
simconf.tstart = 0;
simconf.tstop = 10.0;
simconf.tstep = 5.0e-3;
simconf.fps = 90.0;

% Execute the simlation engine
robotsim.run(simconf);

%%
% EOF
