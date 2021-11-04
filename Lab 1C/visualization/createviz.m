%% Clear workspace

clear all; clear classes; close all; clc;

%% Create a class instance

h = IRB120Visualization;

%% Save to MAT File

% Generate file and directory paths
fname = mfilename;
fpath = mfilename('fullpath');
dpath = strrep(fpath, fname, '');

% Store generated model in the appropriate directory
save(strcat(dpath,'IRB120Model'), 'h');

% Cleanup
close all; clear all; clear classes; clc;

%%
% EOF
