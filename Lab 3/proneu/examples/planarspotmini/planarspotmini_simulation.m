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
%   File:           planarspotmini_simulation.m
%
%   Authors:        Vassilios Tsounis, tsounisv@ethz.ch
%
%   Date:           3/11/2017
%
%   Desciption:     Simulation instance.
%

% Optional calls for workspace resetting/cleaning
close all; clear all; clear classes; clc;

%% Load the Model Data

% Load robotic systm model
load('PlanarSpotMiniModel.mat');

% Load world/enironemnt model
load('PlanarSpotMiniWorld.mat');

%% Create a Robot Controller

% Define the actuator actions.
controller = RobotController(@planarspotmini_controller);

%% Generate the Simulation Environment

% Create the robot simulation engine
robotsim = RobotSimulator(robotmdl, controller, worldmdl, 'solver', 'fixedstep');

%% Set the Model Parameters

% System parameter values
% params = [grav, h_B, l_B, l_FS, l_FT, l_HA, l_LA, l_RS, l_RT, l_UA, m_B, m_FS, m_FT, m_HA, m_LA, m_RS, m_RT, m_UA, r_EE, r_FF, r_FT,  r_HA,  r_LA,  r_RF, r_RT,  r_UA,  w_B]
mparams  = [9.81  0.1  0.6  0.3   0.3   0.15   0.3   0.3   0.3   0.3  10.0 0.5   0.5   0.1   0.5   0.5   0.5   0.5   0.02  0.02  0.015  0.015  0.015  0.02  0.015  0.015  0.2].';

% Store the parameters
robotmdl.parameters.values = mparams;

%% Create Visualizations

% Configurations
fontsize = 12;
csfscale = 0.1;
posoffset = [0; 0; 0.5];
zoom = 2.0;

% Generate 3D Visualization instance
robotviz = RobotVisualization(robotmdl,worldmdl,fontsize,csfscale,zoom,posoffset);
robotviz.open();
robotviz.load();

% Give the simulator access to the 3D visualization
robotsim.setvisualizer(robotviz);

%% Run Simulation

% Set the system's initial conditions
xinit = [0 0.53 0 0.9 -1.5 0.9 -1.7 0.9 0.7 0.4 0 0 0.5 zeros(1,7)].';

% Set the system parameters used by the simulation
robotsim.setup('xinit', xinit);

%
simconf.tstart = 0;
simconf.tstop = 30.0;
simconf.tstep = 1e-4;
simconf.fps = 30.0;

% Execute the simlation engine
robotsim.run(simconf);

%%
% EOF
