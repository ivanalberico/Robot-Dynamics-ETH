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

classdef RigidBodyDescription_v1
    
    % Body descriptor
    properties
        tree        = struct('parent', 0);
        cs          = struct('P_r_PB', zeros(3,1), 'C_PB', zeros(3,3));
        param       = struct('m', 0, ...
                             'B_Th',  zeros(3,3), ...
                             'B_r_COG', zeros(3,1));
    end
    
    % Body generator
    methods
        
        function [] = generatebody(obj, body)
        
            %%%
            % Check input arguments
            %%%
            
            if ~isa(obj, 'RigidBodyDescription_v1')
                error('Argument is not a RigidBodyDescription_v1 type');
            end
            
            if ~isa(body, 'RigidBody')
                error('Argument is not a RigidBody type');
            end
            
            %%%
            % Set symbolic parameters
            %%%
            
            % Kinematics tree configuration
            body.ktree.parents = obj.tree.parent;
            
            % Parent-to-Body kinematics
            body.kinematics.symbols.T_PiBi           = sym(eye(4));
            body.kinematics.symbols.T_PiBi(1:3,4)    = sym(obj.cs.P_r_PO);
            body.kinematics.symbols.T_PiBi(1:3,1:3)  = sym(obj.cs.A_PB);
            
            % Body inertial parameters
            body.inertia.parameters.m_Bi             = sym(obj.param.m);
            body.inertia.parameters.Bi_Theta_Bi      = sym(obj.param.B_Th);
            
            % Body CoG configuration
            body.kinematics.symbols.T_BiCi         = sym(eye(4));
            body.kinematics.symbols.T_BiCi(1:3,4)  = sym(obj.param.B_r_COG);
            
            %%%
            % If specifications are actually numeric then store them directly
            %%%
            
            if isa(obj.cs.P_r_PO, 'numeric')
                body.specs.T_PiBi(1:3,4) = obj.cs.P_r_PO;
            end
            
            if isa(obj.cs.A_PB, 'numeric')
                body.specs.T_PiBi(1:3,1:3) = obj.cs.A_PB;
            end
            
            if isa(obj.param.B_r_COG, 'numeric')
                body.specs.T_BiCi(1:3,4) = obj.param.B_r_COG;
            end
            
            if isa(obj.param.m, 'numeric')
                body.specs.m_Bi = obj.param.m;
            end
            
            if isa(obj.param.B_Th, 'numeric')
                body.specs.Bi_Theta_Bi = obj.param.B_Th;
            end
            
            %%%
            % Generate inertial properties
            %%%
            
            % Generate inertia tensor if necessary
            if ~isempty(body.inertia.m_Bi) && isempty(body.inertia.Bi_Theta_Bi)
                
                % Generate the inertia tensor using principle axes at the body centroid
                m_Bi = body.inertia.parameters.m_Bi;
                Ci_Theta_Ci = geninertiatensor(body.geometry, m_Bi);
                
                % Set the transformation properties
                C_BiCi = body.kinematics.symbols.T_BiCi(1:3,1:3);
                Ci_r_CiBi = - C_BiCi.' * body.kinematics.symbols.T_BiCi(1:3,4);
                
                % Transform accordingly in order to express the inertia tensor in the body frame
                body.inertia.parameters.Bi_Theta_Bi = steiner(m_Bi, Ci_Theta_Ci, Ci_r_CiBi, C_BiCi);
            
            elseif isempty(body.inertia.Bi_Theta_Bi) && isempty(body.inertia.m_Bi)
                
                % If both inertial paremeters are undefine then just set
                % them to zero - mass is probably ignored for this body
                body.inertia.m_Bi = sym(zeros(1,1));
                body.inertia.Bi_Theta_Bi = sym(zeros(3,3));
            end

        end 
        
    end
end
