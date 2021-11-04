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

classdef RigidBodyDescription_v2
    
    % Body descriptor
    properties
        name        = 'body';
        ktree       = struct('nodeid', 1, 'parents', 0);
        cs          = struct('P_r_PB', sym(zeros(3,1)), 'C_PB', sym(eye(3)));
        param       = struct('m',           [], ...
                             'B_Theta_B', [],  ...
                             'B_r_BCoM',    [], ...
                             'C_BCoM',      []);
        geometry    = struct('type', 'none', ...
                             'issolid', false, ...
                             'params', [], ...
                             'values', [], ...
                             'offsets', struct('r', sym(zeros(3,1)), 'C', sym(zeros(3,3))), ...
                             'color', [0.5 0.5 0.5], ...
                             'alpha', 1.0);
        endeffector = false;
    end
    
    % Body generator
    methods
        
        function [] = generatebody(obj, body)
            
            %%%
            % Check input arguments
            %%%
            
            if ~isa(obj, 'RigidBodyDescription_v2')
                error('Argument is not a RigidBodyDescription_v2 type');
            end
            
            if ~isa(body, 'RigidBody')
                error('Argument is not a RigidBody type');
            end
            
            %%%
            % Set symbolic parameters
            %%%
            
            % Kinematics tree configuration
            body.ktree.nodeid    = obj.ktree.nodeid;
            body.ktree.parents   = obj.ktree.parents;
            
            % Parent-to-Body kinematics
            body.kinematics.symbols.T_PiBi           = sym(eye(4));
            if ~isempty(obj.cs.P_r_PB)
                body.kinematics.symbols.T_PiBi(1:3,4)    = sym(obj.cs.P_r_PB);
            end
            if ~isempty(obj.cs.C_PB)
                body.kinematics.symbols.T_PiBi(1:3,1:3)  = sym(obj.cs.C_PB);
            end
            
            % Body CoG configuration
            body.kinematics.symbols.T_BiCi                = sym(eye(4));
            if ~isempty(obj.param.B_r_BCoM)
                body.kinematics.symbols.T_BiCi(1:3,4)     = sym(obj.param.B_r_BCoM);
            end
            if ~isempty(obj.param.C_BCoM)
                body.kinematics.symbols.T_BiCi(1:3,1:3)   = sym(obj.param.C_BCoM);
            end
            
            % Body inertial parameters
            body.inertia.parameters.m_Bi = sym(obj.param.m);
            body.inertia.parameters.Bi_Theta_Bi = sym(obj.param.B_Theta_B);
            
            % Body geometry definitions
            body.geometry.type       = obj.geometry.type;
            body.geometry.issolid    = obj.geometry.issolid;
            body.geometry.params     = obj.geometry.params;
            body.geometry.values     = obj.geometry.values;
            body.geometry.offsets.r  = obj.geometry.offsets.r;
            body.geometry.offsets.C  = obj.geometry.offsets.C;
            body.geometry.color      = obj.geometry.color;
            body.geometry.alpha      = obj.geometry.alpha;
            
            % Body name
            body.name = obj.name;
            
            % Is this an end-effector?
            %body.endeffector = obj.endeffector;
            
            %%%
            % If specifications are actually numeric then store them directly
            %%%
            
            if ~isempty(obj.cs.P_r_PB) && isa(obj.cs.P_r_PB, 'numeric')
                body.specs.T_PiBi(1:3,4) = obj.cs.P_r_PB;
            end
            
            if ~isempty(obj.cs.C_PB) && isa(obj.cs.C_PB, 'numeric')
                body.specs.T_PiBi(1:3,1:3) = obj.cs.C_PB;
            end
            
            if ~isempty(obj.param.B_r_BCoM) && isa(obj.param.B_r_BCoM, 'numeric')
                body.specs.T_BiCi(1:3,4) = obj.param.B_r_BCoM;
            else
                body.specs.T_BiCi(1:3,4) = zeros(3,1);
            end
            
            if  ~isempty(obj.param.C_BCoM) && isa(obj.param.C_BCoM, 'numeric')
                body.specs.T_BiCi(1:3,1:3) = obj.param.C_BCoM;
            else
                body.specs.T_BiCi(1:3,1:3) = eye(3);
            end
            
            if ~isempty(obj.param.m) && isa(obj.param.m, 'numeric')
                body.specs.m_Bi = obj.param.m;
            else
                body.specs.m_Bi = 0;
            end
            
            if ~isempty(obj.param.B_Theta_B) && isa(obj.param.B_Theta_B, 'numeric')
                body.specs.Bi_Theta_Bi = obj.param.B_Theta_B;
            else
                body.specs.Bi_Theta_Bi = zeros(3,3);
            end
            
            %%%
            % Generate inertial properties (if unspecified in description)
            %%%
            
            % Case 1: if m is specified, Theta unspecified -> try and
            % generate from the body's geometry
            if ~isempty(body.inertia.parameters.m_Bi) && isempty(body.inertia.parameters.Bi_Theta_Bi)
                
                % Generate the inertia tensor using principle axes at the body centroid
                m_Bi = body.inertia.parameters.m_Bi;
                Ci_Theta_Ci = geninertiatensor(body.geometry, m_Bi);
                
                % Set the transformation properties
                C_BiCi = body.kinematics.symbols.T_BiCi(1:3,1:3);
                Ci_r_CiBi = - C_BiCi.' * body.kinematics.symbols.T_BiCi(1:3,4);
                
                % Transform accordingly in order to express the inertia tensor in the body frame
                body.inertia.parameters.Bi_Theta_Bi = steiner(m_Bi, Ci_Theta_Ci, Ci_r_CiBi, C_BiCi);
            
            % Case 2: If both m and Theta are unspecified, then the body is
            % massless and we set these to zero. 
            elseif isempty(body.inertia.parameters.Bi_Theta_Bi) && isempty(body.inertia.parameters.m_Bi)
                
                % If both inertial paremeters are undefine then just set
                % them to zero - mass is probably ignored for this body
                body.inertia.parameters.m_Bi = sym(zeros(1,1));
                body.inertia.parameters.Bi_Theta_Bi = sym(zeros(3,3));
            end
            
        end
        
    end
    
end
