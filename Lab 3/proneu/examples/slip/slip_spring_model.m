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
%   File:           monopod_jointspace_pid_controller.m
%
%   Authors:        Vassilios Tsounis, tsounisv@ethz.ch
%
%   Date:           1/2/2017
%
%   Desciption:     Joint-space PID motion controller.
%

function [ F_Sbf ] = slip_spring_model(model,world,logger,t,x)
    
    %params = model.parameters.values;
    qdata = x(1:8);
    dqdata = x(9:15);
    
    % spring deflection
    zeta_S = qdata(8);
    dzeta_S = dqdata(7);
     
    % parameters
    Ks = 500.0;
    Ds = 0.1;
    zeta_0 = 0.3;
    
    % Compute total spring force
    F_Sbf = Ks * (zeta_S - zeta_0) + Ds*dzeta_S;
    
end
