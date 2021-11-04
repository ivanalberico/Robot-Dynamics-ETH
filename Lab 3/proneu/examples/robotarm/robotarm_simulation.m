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
%   File:           robotarm_simulation.m
%
%   Authors:        Vassilios Tsounis, tsounisv@ethz.ch
%
%   Date:           10/12/2016
%
%   Desciption:     Simulation of a planar cart-pole system.
%

% Optional calls for workspace resetting/cleaning
close all; clear all; clear classes; clc;

%% System Configurations

kinematics_only = true;

%% Load the Model Data

% Load robotic systm model
load('RobotArmModel.mat');

% Load world/enironemnt model
load('RobotArmWorld.mat');

%% Create a Robot Controller

% Define the actuator actions.
if kinematics_only == true
    controller = RobotController(@robotarm_kinematics_controller);
else
    controller = RobotController(@robotarm_dynamics_controller);
end

%% Generate the Simulation Environment

% Create the robot simulation engine
if kinematics_only == true
    robotsim = RobotSimulator(robotmdl, controller, worldmdl, 'solver', 'kinematics');
else
    robotsim = RobotSimulator(robotmdl, controller, worldmdl, 'solver', 'dormandprince');
end

%% Set the Model Parameters

% System parameter values
% params = [d_1x, d_1y, d_1z, d_2x, d_2y, d_2z, d_3x, d_3y, d_3z, d_Bx, d_By, d_Bz, grav, h_B, l_1, l_2, l_3, l_B, m_1, m_2, m_3, m_B, r_1, r_2, r_3, w_B]
mparams  = [0.0   0.0   0.2   0.0   0.0   0.5   0.0   0.0   0.5   0.0   0.0   0.0   9.81  0.2  0.4  1.0  1.0  0.3  1.0  1.0  1.0  3.0  0.07 0.05 0.05 0.3].';

% Store the parameters
robotmdl.parameters.values = mparams;

%% Create Visualizations

% Configurations
fontsize = 15;
csfscale = 0.3;
posoffset = [0; 0; 1.0];
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
xinit = [-pi/2 pi/8 pi/8 0.0 0.0 0.0].';

% Set the system parameters used by the simulation
robotsim.setup('xinit', xinit);

% Set simulation time configurations
simconf.tstart = 0;
simconf.tstop = 10.0;
simconf.tstep = 5e-3;
simconf.fps = 30.0;

% Execute the simlation engine
robotsim.run(simconf);

%%
% EOF
