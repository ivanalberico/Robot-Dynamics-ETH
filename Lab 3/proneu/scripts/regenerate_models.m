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
%   File:           regenerate_models.m
%
%   Authors:        Vassilios Tsounis, tsounisv@ethz.ch
%
%   Date:           9/4/2017
%
%   Desciption:     Helper script to automate rebuilding all models to
%                   update the repo/package defaults.
%

%% Example 1 - Cart-Pole

close all; clear all; clear classes; clc;
cartpole_dynamics;
cartpole_world;

%% Example 2 - Double Cart-Pole

close all; clear all; clear classes; clc;
doublecartpole_dynamics;
doublecartpole_world;

%% Example 3 - Robot-Arm

close all; clear all; clear classes; clc;
robotarm_dynamics;
robotarm_world;

%% Example 4 - Constrained Monopod Control

close all; clear all; clear classes; clc;
monopod_dynamics;
monopod_world;

%% Example 5 - Quadrotor Control

close all; clear all; clear classes; clc;
quadrotor_dynamics;
quadrotor_world;

%% Example 6 - SLIP Locomotion

close all; clear all; clear classes; clc;
slip_dynamics;
slip_world;

%% Example 7 - Space-Arm

% close all; clear all; clear classes; clc;
% spacearm_dynamics;
% spacearm_world;

%%
% EOF
