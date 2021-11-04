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

classdef ForceTorqueDescription_v1
    properties
        type    = 'none';
        body_P  = 0;
        body_B  = 0;
        P_r_R   = sym(zeros(3,1));
        B_r_A   = sym(zeros(3,1));
        B_F     = [];
        B_T     = [];
    end
    
    methods
        
        function [] = generateforcetorque(obj, force)
       
            %%%
            % Check input arguments
            %%%
            
            if ~isa(obj, 'ForceTorqueDescription_v1')
                error('Argument is not a ForceTorqueDescription_v1 type');
            end
            
            if ~isa(force, 'ForceTorque')
                error('Argument is not a ForceTorque type');
            end
            
            %%%
            % Set symbolic parameters
            %%%
            
            % Set the kinematic tree parameters for the forcetorque element
            force.ktree.parent    = obj.body_P;
            force.ktree.child     = obj.body_B;
            
            % Configure the element propertis depending on 
            switch obj.type
                
                case 'lin'
                    % Set the force properties
                    force.type                = 'force';
                    force.symbols.Bi_F_j      = obj.B_F;
                    % Set the points of application
                    force.symbols.Pi_r_PiRj   = obj.P_r_R;
                    force.symbols.Bi_r_BiAj   = obj.B_r_A;
                    % Set all torque properties to zero
                    force.symbols.Bi_T_j      = sym(zeros(3,1));
                    
                case 'rot'
                    % Set the torque properties
                    force.type                = 'torque';
                    force.symbols.Bi_T_j      = obj.B_T;
                    % Set all force properties to zero
                    force.symbols.Bi_r_BiAj   = sym(zeros(3,1));
                    force.symbols.Pi_r_PiRj   = sym(zeros(3,1));
                    force.symbols.Bi_F_j      = sym(zeros(3,1));
                
                otherwise
                    error('Incorrect type specified for force-torque element.');
            end
            
        end
        
    end
end
