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

function [ G_A, G_B ] = getRotationRateMapCardanXyz(phi)
    
    % Intilize rate matrix
    if isa(phi,'sym')
        G_A = sym(zeros(3,3));
    else
        G_A = zeros(3,3);
    end

    % Extract the parametrization
    x = phi(1);
    y = phi(2);
    z = phi(3);
    
    % Compute the full rotation matrix
    C_AB = getRotationMatrixX(x)*getRotationMatrixY(y)*getRotationMatrixZ(z);
    
    % Compute G matrix in origin frame A
    G_A(1:3,1) = [1;              0;        0             ];
    G_A(1:3,2) = [0;              cos(x);   sin(x)        ];
    G_A(1:3,3) = [sin(y);  -cos(y)*sin(x);  cos(x)*cos(y) ];
    
    % Compute G matrix in target frame B
    G_B = C_AB.' * G_A;
    
    % Simplify symbolic matrices
    if isa(phi,'sym')
        G_A = simplify(G_A);
        G_B = simplify(G_B);
    end

end
