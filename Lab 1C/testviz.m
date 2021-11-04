%% Load the visualization

% Clear exisint figures
close all; clear all; clear classes; clc;

% Run the loading script
loadviz;

%% Run the test visualization motion

% Initialize a vector of generalized coordinates
q = zeros(6,1);

% Set the sampling time (in seconds)
ts = 0.02;

% Set the duration of the visualization (in seconds)
tf = 10.0;
kf = tf/ts; % Number of iterations as a function of the duration and the sampling time

% Notify that the visualization loop is starting
disp('Starting visualization loop.');

% Run a visualization loop
for k=1:kf
    try
        % Start a timer
        startLoop = tic;
        % Set a desired vector of generalized coordinates.
        % We use a discretized version of the sine function.
        q = 0.2*sin(2*pi*0.5*k*ts) * ones(6,1);
        % Set the generalized coordinates to the robot visualizer class
        abbRobot.setJointPositions(q);
        % Update the visualization figure
        drawnow;
        % If enough time is left, wait to try to keep the update frequency
        % stable
        residualWaitTime = ts - toc(startLoop);
        if (residualWaitTime > 0)
            pause(residualWaitTime);
        end
    catch
        disp('Exiting the visualization loop.');
        break;
    end
end

% Notify the user that the script has ended.
disp('Visualization loop has ended.');

%% 
% EOF