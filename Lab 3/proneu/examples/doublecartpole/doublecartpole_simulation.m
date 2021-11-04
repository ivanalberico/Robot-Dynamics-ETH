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
%   File:           doublecartpole_simulation.m
%
%   Authors:        Vassilios Tsounis, tsounisv@ethz.ch
%
%   Date:           10/12/2016
%
%   Desciption:     Simulation of a planar cart-pole system.
%

% Optional calls for workspace resetting/cleaning
close all; clear all; clear classes; clc;

%% Simulation Configurations

use_legacy_proneu = false;

%% Load the Model Data

% Load robotic systm model
load('DoubleCartPoleModel.mat');

% Load world/enironemnt model
load('DoubleCartPoleWorld.mat');

%% Create a Robot Controller

% Define the actuator actions.
controller = RobotController(@doublecartpole_controller);

%% Generate the Simulation Environment

% Create the robot simulation engine
robotsim = RobotSimulator(robotmdl, controller, worldmdl, 'solver', 'dormandprince');

%% Set the Model Parameters

% System parameter values
% params = [d_Cx, d_Cy, d_Cz, d_Px, d_Py, d_Pz, d_Qx, d_Qy, d_Qz, grav, h_C, l_C, l_P, l_Q, m_C, m_P, m_Q, r_P, r_Q, w_C]
mparams  = [0.0   0.0   0.0   0.0   0.0   0.5   0.0   0.0   0.5   9.81  0.2  0.3  1.0  1.0  3.0  0.2  0.2  0.05 0.05 0.3].';

% Store the parameters
robotmdl.parameters.values = mparams;

%% Create Visualizations

% Configurations
fontsize = 15;
csfscale = 0.3;
posoffset = [0; 0; 0.0];
zoom = 1.0;

% Generate 3D Visualization instance
robotviz = RobotVisualization(robotmdl,worldmdl,fontsize,csfscale,zoom,posoffset);
robotviz.open();
robotviz.load();

% Give the simulator access to the 3D visualization
robotsim.setvisualizer(robotviz);

%% Adapt v2 to v1 output

% Load robotic systm model
if use_legacy_proneu == true
    robotmdl.dynamics.symbols.tau = robotmdl_legacy.fpNE;
    robotmdl.dynamics.symbols.g = robotmdl_legacy.gpNE;
    robotmdl.dynamics.symbols.b = robotmdl_legacy.bpNE;
    robotmdl.dynamics.symbols.M = robotmdl_legacy.MpNE;
end

%% Run Simulation

% Set the system's initial conditions
xinit = [0.0 pi/4 0.0 0.0 0.0 0.0].';

% Set the system parameters used by the simulation
robotsim.setup('xinit', xinit);

%
simconf.tstart = 0;
simconf.tstop = 10.0;
simconf.tstep = 5e-3;
simconf.fps = 30.0;

% Execute the simlation engine
robotsim.run(simconf);

%%
% EOF
