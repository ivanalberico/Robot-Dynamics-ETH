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
%
%   PARAMETRIZATIONS OF 3D ORIENTATIONS
%
%   1. Quaternion                           : phi_IB = [p_B0 p_B1 p_B2 p_B3].'
%
%   2. Axis-Angle & Rotation Vector         : phi_IB = [theta_B n_Bx n_By n_Bz].'
%
%   3. Euler Angles:
%
%       3.1 Proper ZYZ                      : phi_IB = [alpha_B beta_B gamma_B].'
%
%       3.2 Proper ZXZ                      : phi_IB = [alpha_B beta_B gamma_B].'
%
%       3.3 Tait-Bryan (ZYX)                : phi_IB = [theta_Bz theta_By theta_Bx].'
%
%       3.4 Cardan (XYZ)                    : phi_IB = [theta_Bx theta_By theta_Bz].'
%
%
%   IDENTIFIERS:
%
%   'quaternion', 'angleaxis', 'eulerzyz', 'eulerzxz', 'taitbryanzyx',
%   'cardanxyz', 'custom'
%

function [ r_IB, phi_IB, C_IB, dphi_IB, v_IB, omega_IB, F_I, F_B, G_I, G_B] = genfltbody(orientation, varargin)

    % Assume default name
    name = 'B';
    
    % Check for extra arguments
    if ~isempty(varargin)
        for i=1:length(varargin) 
            if isa(varargin{i}, 'char') && strcmp(varargin{i}, 'name')
               name = varargin{i+1}; 
            end
        end
    end

    % Generate the base position coordinates and vector
    r_IB = generate_position_vector(name);

    % Generate the body orientation using a specific paramatrization
    switch orientation
        case 'quaternion'
            phi_IB = generate_rotation_quaternion(name);
            C_IB = mapQuaternionToRotationMatrix(phi_IB);
        
        case 'angleaxis'
            phi_IB = generate_angle_axis(name);
            C_IB = mapAngleAxisToRotationMatrix(phi_IB(1), phi_IB(2:4));
        
        case 'taitbryanzyx'
            phi_IB = generate_taitbryan_euler_angles(name);
            C_IB = mapEulerAnglesZyxToRotationMatrix(phi_IB);
        
        case 'cardanxyz'
            phi_IB = generate_cardan_euler_angles(name);
            C_IB = mapEulerAnglesXyzToRotationMatrix(phi_IB);
        
        case 'eulerzyz'
            phi_IB = generate_proper_euler_angles(name);
            C_IB = mapEulerAnglesZyzToRotationMatrix(phi_IB);
        
        case 'eulerzxz'
            phi_IB = generate_proper_euler_angles(name);
            C_IB = mapEulerAnglesZxzToRotationMatrix(phi_IB);
        
        otherwise
            warning('A parametrization for base orientation was not specified. Defaulting to unit quaternions.');
    end
    
    % Generate derivative symbols for orientation
    dphi_IB = genderivsym(phi_IB);
    
    % Generate the linear ang angular velocity vectors
    v_IB = genderivsym(r_IB);
    omega_IB = generate_angular_velocity(name);

    % Compute the F matrix appropriately
    [F_I, F_B] = getInverseRotationRateMap(orientation, phi_IB);
    [G_I, G_B] = getRotationRateMap(orientation, phi_IB);
end

%%%
%
%   HELPER FUNCTIONS
%
%%%

function [ phi ] = generate_rotation_quaternion(name)
    syms phi;
    syms(strcat('p_',name,'w'),strcat('p_',name,'x'),strcat('p_',name,'y'),strcat('p_',name,'z'));
    eval(['phi = [' strcat('p_',name,'w') ' ' strcat('p_',name,'x') ' ' strcat('p_',name,'y') ' ' strcat('p_',name,'z') '];']);
    phi = phi(:);
    assume(phi,'real');
end

function [ phi ] = generate_proper_euler_angles(name)
    syms phi;
    syms(strcat('alpha_',name),strcat('beta_',name),strcat('gamma_',name));
    eval(['phi = [' strcat('alpha_',name) ' ' strcat('beta_',name) ' ' strcat('gamma_',name) '];']);
    phi = phi(:);
    assume(phi,'real');
end

function [ phi ] = generate_taitbryan_euler_angles(name)
    syms phi;
    syms(strcat('theta_',name,'z'),strcat('theta_',name,'y'),strcat('theta_',name,'x'));
    eval(['phi = [' strcat('theta_',name,'z') ' ' strcat('theta_',name,'y') ' ' strcat('theta_',name,'x') '];']);
    phi = phi(:);
    assume(phi,'real');
end

function [ phi ] = generate_cardan_euler_angles(name)
    syms phi;
    syms(strcat('theta_',name,'x'),strcat('theta_',name,'y'),strcat('theta_',name,'z'));
    eval(['phi = [' strcat('theta_',name,'x') ' ' strcat('theta_',name,'y') ' ' strcat('theta_',name,'z') '];']);
    phi = phi(:);
    assume(phi,'real');
end

function [ phi ] = generate_angle_axis(name)
    syms phi;
    syms(strcat('theta_',name),strcat('n_x',name,'x'),strcat('n_',name,'y'),strcat('n_',name,'z'));
    eval(['phi = [' strcat('theta_',name) ' ' strcat('n_',name,'x') ' ' strcat('n_',name,'y') ' ' strcat('n_',name,'z') '];']);
    phi = phi(:);
    assume(phi,'real');
end

function [ omega ] = generate_angular_velocity(name)
    syms omega;
    syms(strcat('omega_',name,'x'),strcat('omega_',name,'y'),strcat('omega_',name,'z'));
    eval(['omega = [' strcat('omega_',name,'x') ' ' strcat('omega_',name,'y') ' ' strcat('omega_',name,'z') '];']);
    omega = omega(:);
    assume(omega,'real');
end

function [ r ] = generate_position_vector(name)
    syms r;
    syms(strcat('x_',name),strcat('y_',name),strcat('z_',name));
    eval(['r = [' strcat('x_',name) ' ' strcat('y_',name) ' ' strcat('z_',name) '];']);
    r = r(:);
    assume(r,'real');
end
