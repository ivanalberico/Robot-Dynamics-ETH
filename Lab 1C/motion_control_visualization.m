function [] = motion_control_visualization()
% Motor control visualization script

% ============== Trajectory settings ======================
ts = 0.05;                          % Set the sampling time (in seconds)
r_start = [0.4 0.1 0.6].';          % 3x1 (m)
r_end = [-0.4 0.3 0.5].';           % 3x1 (m)
v_line = 0.4;                       % 1x1 (m/s)
q_0 = zeros(6,1);                   % 6x1 (rad)
use_solution = 0;                   % 0: user implementation, 1: solution               
% =========================================================

% Load the visualization
f1 = figure(1); close(f1); loadviz;

% Initialize the vector of generalized coordinates
q = q_0;
abbRobot.setJointPositions(q);

% Generate a new desired trajectory
dr = r_end - r_start;
tf = norm(dr)/v_line; % Total trajectory time
N = floor(tf/ts);           % Number time steps
t = ts*1:N;
r_traj = generateLineTrajectory(r_start, r_end, N); 
v_traj = repmat(v_line * (dr).'/norm(dr), N, 1); % Constant velocity reference
r_log = NaN*zeros(size(r_traj));
v_log = NaN*zeros(size(v_traj));

% Plot real trajectory
figure(2); clf; hold all
r_h = plot(t, r_log);
for i = 1:3
    plot(t, r_traj(:,i), '--', 'Color', get(r_h(i),'Color'));
end
title('End effector position in Inertial frame')
legend({'x','y','z','x_{ref}','y_{ref}','z_{ref}'})

figure(3); clf; hold all
v_h = plot(t, v_log);
for i = 1:3
    plot(t, v_traj(:,i), '--', 'Color', get(r_h(i),'Color'));
end
title('End effector linear velocity in Inertial frame')
legend({'x','y','z','x_{ref}','y_{ref}','z_{ref}'})

% Notify that the visualization loop is starting
disp('Starting visualization loop.');
pause(0.5);

% Run a visualization loop
for k = 1:N
    startLoop = tic; % start time counter
    
    % Get the velocity command
    switch use_solution
        case 0
            Dq = kinematicMotionControl(q, r_traj(k,:).', v_traj(k,:).');
        case 1
            Dq = kinematicMotionControl_solution(q, r_traj(k,:).', v_traj(k,:).');
    end
    
    % Time integration step to update visualization. This would also be used for a position controllable robot
    q = q + Dq*ts;
    
    % Set the generalized coordinates to the robot visualizer class
    abbRobot.setJointPositions(q);
    r_log(k,:) = jointToPosition_solution(q);
    v_log(k,:) = jointToPosJac_solution(q)*Dq;
    
    % Update the visualizations
    for i = 1:3
        set(r_h(i), 'Ydata', r_log(:,i));
        set(v_h(i), 'Ydata', v_log(:,i));
    end
    drawnow;
    
    % If enough time is left, wait to try to keep the update frequency
    % stable
    residualWaitTime = ts - toc(startLoop);
    if (residualWaitTime > 0)
        pause(residualWaitTime);
    end
end

% Notify the user that the script has ended.
disp('Visualization loop has ended.');


end

function [ r_traj ] = generateLineTrajectory(r_start, r_end, N)
% Inputs: 
%		r_start : start position
%		r_end   : end position
%		N       : number of timesteps
% Output: Nx3 matrix End-effector position reference
x_traj = linspace(r_start(1), r_end(1), N);
y_traj = linspace(r_start(2), r_end(2), N);
z_traj = linspace(r_start(3), r_end(3), N);
r_traj = [x_traj; y_traj; z_traj].';
end

