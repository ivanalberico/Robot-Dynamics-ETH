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
%   File:           test_models.m
%
%   Authors:        Vassilios Tsounis, tsounisv@ethz.ch
%
%   Date:           26/4/2017
%
%   Desciption:     Helper script to automate rebuilding all models to
%                   update the repo/package defaults.
%

% Set output screen position
% set(0,'DefaultFigurePosition', [-1920 1080 1920 1080]);
set(0,'DefaultFigurePosition', get(0, 'ScreenSize'));

%% Example 1 - Cart-Pole

close all; clear all; clear classes; clc;
cartpole_simulation;

%% Example 2 - Double Cart-Pole

close all; clear all; clear classes; clc;
doublecartpole_simulation;

%% Example 3 - Robot-Arm

close all; clear all; clear classes; clc;
robotarm_simulation;

%% Example 4 - Constrained Monopod Control

close all; clear all; clear classes; clc;
monopod_simulation;

%% Example 5 - Quadrotor Control

close all; clear all; clear classes; clc;
quadrotor_simulation;

%% Example 6 - SLIP Locomotion

close all; clear all; clear classes; clc;
slip_simulation;

%% Example 7 - Planer Spot Mini

close all; clear all; clear classes; clc;
planarspotmini_simulation;

%%
% EOF
