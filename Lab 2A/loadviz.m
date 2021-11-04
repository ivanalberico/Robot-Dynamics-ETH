%% Load the visualization MAT file

% Load visualization files.
load IRB120Model.mat
abbRobot = h; clear h;
abbRobot.load(0.1, 11, [0.0;0.0;0]);

%% Configure the visualization

% Set lighting.
lightangle(-120,  30);
lightangle( 120, -30);
lightangle(-120, -30);
lightangle( 120,  30);
lightangle(-120,  30);
lightangle( 120, -30);
lightangle(-120, -30);
lightangle( 120,  30);

% Set view angle.
view(24,33);
set(gca,'CameraPosition',  1e-3*[2679.17 -4731.06 3838.21], ...
        'CameraTarget',    1e-3*[381.33   429.964 169.419], ...
        'CameraViewAngle', 11.3725);

%%
% EOF
