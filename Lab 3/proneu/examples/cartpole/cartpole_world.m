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
%   File:           doublecartpole_world.m
%
%   Authors:        Vassilios Tsounis, tsounisv@ethz.ch
%
%   Date:           10/12/2016
%
%   Desciption:     Simulation of a planar cart-pole system.
%

% Optional calls for workspace resetting/cleaning
close all; clear all; clear classes; clc;

%% Define World Model Parameters

% Floor parameters
syms w_floor l_floor real;

% Rail parameters
syms l_rail w_rail h_rail real;

%%

k=1;
element(k) = WorldElementDescription;
element(k).name = 'floor';
element(k).geometry.type = 'plane';
element(k).geometry.issolid = false;
element(k).geometry.params = [w_floor l_floor]; 
element(k).geometry.values = [10.0 10.0];
element(k).geometry.offsets.r = [0 0 1.1].';
element(k).geometry.offsets.C = eye(3);
element(k).geometry.color = [0.96 0.96 0.96];

k=2;
element(k) = WorldElementDescription;
element(k).name = 'rail';
element(k).geometry.type = 'cuboid';
element(k).geometry.issolid = false;
element(k).geometry.params = [l_rail w_rail h_rail]; 
element(k).geometry.values = [10.0 0.05 0.1];
element(k).geometry.offsets.r = [0 -0.15 -0.10].';
element(k).geometry.offsets.C = eye(3);
element(k).geometry.color = [0.3 0.3 0.3];

%% Generate a Model of the World

% Set the world's name
world_name = 'CartPoleWorld';

% Define an empty disturbance function
distfunc = @(model,logger,t,x)[0];

% Generate the world model object
worldmdl = WorldModel(element, distfunc, 'name', world_name);

%% Save to MAT File

% Generate file and directory paths
fname = mfilename;
fpath = mfilename('fullpath');
dpath = strrep(fpath, fname, '');

% Store generated model in the appropriate directory
save(strcat(dpath,worldmdl.name), 'worldmdl');

%%
% EOF
