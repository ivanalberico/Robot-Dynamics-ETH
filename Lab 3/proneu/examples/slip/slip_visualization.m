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
%   File:           slip_visualization.m
%
%   Authors:        Vassilios Tsounis, tsounisv@ethz.ch
%
%   Date:           10/12/2016
%
%   Desciption:     Visualization of a 3D SLIP model.
%

% Optional calls for workspace resetting/cleaning
close all; clear all; clear classes; clc;

%% Load the Model Data

load('SlipModel.mat');
load('SlipWorld.mat');

%% Set the Numerical Parameters

% System parameter values
% params = [ grav, l_Sb, l_Sf, m_B, m_F,   r_B, r_F, r_Sb, r_Sf]
params   = [0.0    0.5   0.5   1.0  0.001  0.2  0.07 0.05  0.05].';

% Store the parameters
robotmdl.parameters.values = params;

%% Create the Visualizations

% Generate 3D Visualization instance
clc
robotviz = RobotVisualization(robotmdl,worldmdl);
robotviz.open();
robotviz.load();

%%
% EOF
