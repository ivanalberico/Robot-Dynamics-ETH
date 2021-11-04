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
% 

%%
%   File:           quadrotor_world_environemnt.m
%
%   Authors:        Vassilios Tsounis, tsounisv@ethz.ch
%
%   Date:           10/12/2016
%
%   Desciption:     An example for implementing external disturances acting
%                   on a quadrotor.
%

function [ W_E ] = quadrotor_world_environemnt(model,logger,t,x)
    
    % Default values
    F_Ex = 0.0;
    F_Ey = 0.0;
    T_Ex = 0.0;
    T_Ey = 0.0;
      
    % Impulsive force/torque example
    if (t>2.0) && (t<=2.1)
        F_Ez = -1.5;
    else
        F_Ez = 0.0;
    end
    
    % Add noise - TODO: add this such as to simulate disturbance wind
    T_Ez = normrnd(0, 0.1);
  
    % Set total external wrench
    W_E = [F_Ex F_Ey F_Ez T_Ex T_Ey T_Ez].';
    
end
