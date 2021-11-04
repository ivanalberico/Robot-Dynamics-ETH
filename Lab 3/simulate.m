%% Setup Simulation

try % Try if the visualization instance is still open
    
    % Initialize the state in the visualization.
    robotviz.update(xinit(1:10), zeros(10,1), zeros(3,1));
    
catch % If it doesn't work, open a new visualization instance
   
    % Clear the entire MATLAB workspace     
    close all; clearvars; clear classes;
    
    % Load the pre-generated model of the robot
    load('LeggedRobotModel.mat'); 
    load('GroundAndWallWorld.mat');
    
    % Set model parameters
    % Def:     [grav, h_B, l_B, l_FS, l_FT, l_HA, l_LA, l_RS, l_RT, l_UA, m_B, m_FS, m_FT, m_HA, m_LA, m_RS, m_RT, m_UA, r_EE, r_FF, r_FT,  r_HA,  r_LA,  r_RF, r_RT,  r_UA,  w_B]
    mparams  = [9.81  0.1  0.6  0.3   0.3   0.15  0.3   0.3   0.3   0.3   10.0 0.5   0.5   0.1   0.5   0.5   0.5   0.5   0.02  0.02  0.015  0.015  0.015  0.02  0.015  0.015  0.2].';
    robotmdl.parameters.values = mparams;
    
    % Visualization configurations
    fontsize = 10;
    csfscale = 0.1;
    posoffset = [0; 0; 0.5];
    zoom = 2.0;
    
    % Generate 3D Visualization instance
    robotviz = RobotVisualization(robotmdl, worldmdl, fontsize, csfscale, zoom, posoffset);
    robotviz.open();
    robotviz.load();
    
end

%% Configure Experiment

% Set the initial state of the system
% Def:  [xb   zb    thb  q1   q2    q3   q4    q5   q6   q7   dot(q)
xinit = [0.0  0.49  0.0  0.9  -1.5  0.9  -1.5  0.5  0.8  0.6  zeros(1,10)].';

% Set distance of wall in positive X-direction in world coordinates.
dx = 0.8;

% Set the controller to use
% controller = @jointspace_pid_control;
% controller = @floating_base_control;
% controller = @floating_base_control_solution;
% controller = @hybrid_force_motion_control;
controller = @hybrid_force_motion_control_solution;

% Control frequency (Hz)
ctrlfreq = 400;

%% Run Simulation

% Initialize the system state and input.
x = xinit;
u = zeros(7,1);

% Sets the total duration of the simulation experiment.
total_sim_time = 30.0;

% Set to < 1.0 for running simulation in slow-motion.
real_time_factor = 1.0;

% Simulation timing configurations.
dt_ctrl = 1/ctrlfreq;   % Control rate (inverse of frequencey)
dt_sim = 1e-3;          % Physics simulation time-step
dt_viz = 1/30;          % Visualization frames-per-second (1/FPS)

% Initializes visualization
robotviz.setcallback(@forceviz);
robotviz.update(x(1:10), zeros(10,1), zeros(3,1));

% Exectue a simulation using a fixed-step integration scheme.
vizT = tic;
totT = tic;
ctrlT = 0;
for t = 0:dt_sim:total_sim_time
    
    % Compute interfaction forces with the wall
    F_EE = wall(robotmdl, x, dx);
    
    % Integrate the system dynamics - updates state at dt_sim increments.
    ddq = ccfd(robotmdl, x, u, F_EE);
    x = x + [x(11:end); ddq]*dt_sim;

    % Update torques from controller - applies zero-order hold.
    if t-ctrlT > dt_ctrl
        u = controller(robotmdl, t, x);
        ctrlT = t;
    end
    
    % Update visualization and configured FPS
    if toc(vizT) > dt_viz * real_time_factor
        robotviz.update(x(1:10), x(11:20), F_EE);
        vizT = tic;
        if t > toc(totT)
            pause(t - toc(totT));
        end
    end
    
end

%% EOF
