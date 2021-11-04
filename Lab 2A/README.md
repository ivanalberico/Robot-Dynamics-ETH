# Robot Dynamics Exercises

## Overview

The first exercise of the Robot Dynamics lecture focuses on the derivation of the Forward Kinematics of a robotic arm.
All functions will be implemented in MATLAB. A tool is provided which visualizes the robot arm in any desired configuration.


## Install


### Step 1. Running Scripts & MATLAB Paths

The scripts which are mentioned in the steps which follow can either be run from the MATLAB console or from within a `*.m` script file.
Please note that in order for these scripts to work properly (including those you will modify), your MATLAB path must be set-up
correctly. This is exactly what the script in Step.2 does, so that you do not need to do it yourself. Please refer to the following link about
[MATLAB Paths](https://www.mathworks.com/help/matlab/matlab_env/what-is-the-matlab-search-path.html) for more information.


### Step 2. Initialize your MATLAB Workspace

To properly initialize your workspace for the exercise, just run the following script from the folder in which this text file is located:

    init_workspace;

This will add the exercise and visualization related folders and sub-folders to the MATLAB path.


### Step 3. Test if Visualization Works

After setting up your workspace, you must test that everything has been set-up correctly. To do this, we can test the visualization, by running
the following script:

    testviz;

This will load the visualization and run a simple motion defined in joint-space.



## Solutions

After implemention your solutions in the template files located in the `problems` folder, you can test them by running the following script:

    evaluate_problems;

This will test all the functions to be implemented in this exercise against their respective solutions.



## Visualization

We provide a simple and modular visualization of the ABB IB120 robotic arm, which enables you to easily inspect the computations of your resulting
kinematic model. This visualization can be loaded once, and then run in the background waiting for new joint states. These can be provided by you,
either from the MATLAB console or from a script. To load the visualization, you can run the following script:

    loadviz;

It will load an object named abbRobot into the MATLAB workspace, which then allows you to visualize the robot in any joint space configuration.
The joint configuration must be a MATALB array of dimension 6x1 (which is a vector of 6 coordinates), and this is then given as an input argument
to the `setJointPositions()` function of the `abbRobot` object.

For example you can run:


    % Generate a random joint state vector (array)
    q = rand(6,1);

    % Provide random joint state to the ABB IB120 object
    abbRobot.setJointPositions(q);


----
Copyright (C) 2017, Robotic Systems Lab, ETH Zurich. - https://www.rsl.ethz.ch
