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

function [ Theta ] = geninertiatensor(geometry, mass)

    % Initialize default inertia tensor
    Theta = zeros(3,3);

    % Retreive parameters
    m = mass;
    bodytype = geometry.type;
    if isfield(geometry, 'issolid')
        issolid  = geometry.issolid;
    else
        issolid = true;
    end

    switch bodytype
        case 'particle'
            Theta = zeros(3,3);

        case 'ellipsoid'
            a = geometry.params(1);
            b = geometry.params(2);
            c = geometry.params(3);
            Theta = [(1/5)*m*(b^2 + c^2) 0 0; 0 (1/5)*m*(a^2 + c^2) 0; 0 0 (1/5)*m*(a^2 + b^2)];

        case 'sphere'
            r = geometry.params(1);
            if issolid == true
                Theta = [(2/5)*m*(r^2) 0 0; 0 (2/5)*m*(r^2) 0; 0 0 (2/5)*m*(r^2)];
            else
                Theta = [(2/3)*m*(r^2) 0 0; 0 (2/3)*m*(r^2) 0; 0 0 (2/3)*m*(r^2)];
            end

        case 'cylinder'
            r = geometry.params(1);
            l = geometry.params(2);
            if issolid == true
                Theta = [(1/12)*m*(3*r^2 + l^2) 0 0; 0 (1/12)*m*(3*r^2 + l^2) 0; 0 0 (1/2)*m*(r^2)];
            else
                Theta = [(1/12)*m*(6*r^2 + l^2) 0 0; 0 (1/12)*m*(6*r^2 + l^2) 0; 0 0 m*(r^2)];
            end

        case 'rod'
            l = geometry.params(1);
            Theta = [(1/12)*m*(l^2) 0 0; 0 (1/12)*m*(l^2) 0; 0 0 0];

        case 'cuboid'
            l = geometry.params(1);
            w = geometry.params(2);
            h = geometry.params(3);
            if issolid == true
                Theta = [(1/12)*m*(w^2 + h^2) 0 0; 0 (1/12)*m*(l^2 + h^2) 0; 0 0 (1/12)*m*(w^2 + l^2)];
            else
                %
                % TODO
                %
            end

        otherwise
            error('proNEu: generateInertiaTensor: Incorrect type specified for body geometry');
    end

end
