% Optional calls for workspace resetting/cleaning
close all; clear all; clear classes; clc;

%% Define World Model Parameters

% Floor parameters
syms w_ground h_ground w_wall h_wall real;

%%

k=1;
element(k) = WorldElementDescription;
element(k).name = 'ground';
element(k).geometry.type = 'plane';
element(k).geometry.issolid = false;
element(k).geometry.params = [w_ground h_ground]; 
element(k).geometry.values = [10.0 10.0];
element(k).geometry.offsets.r = [0 0 0].';
element(k).geometry.offsets.C = eye(3);
element(k).geometry.color = [0.96 0.96 0.96];

k=2;
element(k) = WorldElementDescription;
element(k).name = 'wall';
element(k).geometry.type = 'plane';
element(k).geometry.issolid = false;
element(k).geometry.params = [w_wall h_wall]; 
element(k).geometry.values = [2.0 1.5];
element(k).geometry.offsets.r = [-1.55 0.0 -1.5].';
element(k).geometry.offsets.C = getRotationMatrixY(pi/2);
element(k).geometry.color = [0.5 0.5 0.5];


%% Generate a Model of the World

% Set the world's name
world_name = 'GroundAndWallWorld';

% Generate the world model object
worldmdl = WorldModel(element, @feet_arm_contacts, 'name', world_name);

%% Save to MAT File

% Generate file and directory paths
[fpath,fname,fext] = fileparts(mfilename('fullpath'));
dpath = strrep(fpath, 'generators', 'models/');

% Store generated model in the appropriate directory
save(strcat(dpath,worldmdl.name), 'worldmdl');

%%
% EOF
