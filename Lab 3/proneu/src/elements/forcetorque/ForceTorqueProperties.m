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

classdef ForceTorqueProperties
    
    % Definition quantities
    properties
        % Point of application of the reaction force
        Pi_r_PiRj = [];
        % Point of application of the action force
        Bi_r_BiAj = [];
        % The acting force and torque in the local body frame
        Bi_F_j  = [];
        Bi_T_j  = [];
        % The acting force and torque in the inertial frame
        I_F_j   = [];
        I_T_j   = [];
    end
    
    % Derived quantities
    properties
        % The force-torque Jacobin matrix
        I_J_j   = [];
        % The total contribution to the generalized forces
        tau_j   = [];
    end
end
