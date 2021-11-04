# proNEu v2.0

## Disclaimer

 Please note that this software is currently under active development and highly experimental. Thus it is likely to change frequently and the authors do not guarantee stability of the latest version.

## Overview

**Platforms:** The software works using Mathworks MATLAB **R2017a**, under Windows 10 and Ubuntu 16.04LTS.  
**Authors:** Marco Hutter, Christian Gehring, C. Dario Bellicoso, Vassilios Tsounis  
**Maintainers:** Vassilios Tsounis  
**License:** The source code is released under a [GPL license](LICENSE.txt).

## Description

A simple Matlab Tool for Analytical Derivation of global Kinematics and Dynamics based on projected Newton-Euler methods. The tool proNEu uses the MATLAB Symbolic Math Toolbox to derive the analytical global kinematics and equations of motion based on projected Newton-Euler methods.

## Installation

In order to install, clone the latest version from this repository onto your desktop or laptop and then the directory to your [MATLAB path](https://www.mathworks.com/help/matlab/matlab_env/add-remove-or-reorder-folders-on-the-search-path.html).

## Getting Started

### Demo

The first thing to run, is the demo script in `scripts/test_models.m`. This will load and run all example simulations in sequence. If all goes well then enjoy the demo, else, please post errors in the issue tracker (see below).

### Examples

Once all demos have been run, and seem to work on your system, then proceed to the `examples/`  directory try some of them out. In order to run a full simulation, just open and run the respective `*_simulation.m` for that system. The simulation file loads the model calls object stored in the provided `*.mat` file, defines parameters for this model and proceeds to create an instances of visualizer and simulator objects. If you want to see how the model is generated, open and run the respective `*_dynamics.m` script.

## Documentation

Currently, the new version and API have not been fully documented. However, the manual from the previous version of `proneu.v1` can be found in the `documentation/manual` directory. For the moment this can serve as a sort-of a guide for the new implementation, since much of the same logic is applied for the model generation. We are planning on releasing proper documentation within Q1 of 2018.

## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://bitbucket.org/leggedrobotics/proneu/issues).

----
Copyright (C) 2017 Robotic Systems Lab, ETH Zurich. - https://www.rsl.ethz.ch
