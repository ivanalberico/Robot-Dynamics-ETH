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

classdef RigidBodySpecifications
    properties
        % Kinematic tree configuration.
        T_PiBi   = eye(4);
        % Kinematic tree configuration.
        T_BiCi = eye(4);
        % Inertial parameters
        m_Bi = 0;
        Bi_Theta_Bi = zeros(3,3);
        % Body intrinsic geometric parameters.
        beta_Bi  = [];
        % External parameters the body quantities depend on.
        gamma_Bi = [];
    end
end
