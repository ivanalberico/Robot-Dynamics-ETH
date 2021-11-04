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

load('SlipModel.mat');
load('SlipWorld.mat');

%% Create a Controller

% Generate a robot controller using an external implementation
% controller = RobotController(@slip_spring_model);
controller = RobotController(@slip_raibert_controller);

%% Generate the Simulation Environment

% Create the robot simulation engine
robotsim = RobotSimulator(robotmdl, controller, worldmdl, 'solver', 'fsfb');

% Data Plots
% robotsim.scope.open();

%% Set the Model Parameters

r0      = [-2 0 2.0].';    % Base position
phi0    = [1 0 0 0].';  % Base orientation
qj0     = [0.3].';    % Joint configuration
v0      = [0 0 0].';    % Base linear velocity
omega0  = [0 0 0].';    % Base andular veclocity
dqj0    = [0].';    % Joint velocities

% Initial system state
xinit = [r0; phi0; qj0; v0; omega0; dqj0].';

% System parameter values
% params = [ grav, l_Sb, l_Sf, m_B, m_F,   r_B, r_F, r_Sb, r_Sf]
params   = [9.81   0.5   0.5   0.6  0.001  0.2  0.07 0.05  0.05].';

% Store the parameters
robotmdl.parameters.values = params;

%% Create the Visualizations

% Configurations
fontsize = 12;
csfscale = 0.15;
posoffset = [0; 0; 0.0];
zoom = 0.75;

% Generate 3D Visualization instance
robotviz = RobotVisualization(robotmdl,worldmdl,fontsize,csfscale,zoom,posoffset);
robotviz.open();
robotviz.load();
% set(gcf, 'Position', get(0, 'Screensize'));

% Give the simulator access to the 3D visualization
robotsim.setvisualizer(robotviz);

%% Run Simulation

% Set the system parameters used by the simulation
robotsim.setup('xinit', xinit);

% Simulation time configurations
simconf.tstart  = 0;
simconf.tstop   = 10.0;
simconf.tstep   = 1e-3;
simconf.fps     = 30.0;

% Execute the simlation engine
robotsim.run(simconf);

%%
% EOF
