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

function [ F_A, F_B ] = getInverseRotationRateMapEulerZxz(phi)
    
    % Intilize rate matrix
    if isa(phi,'sym')
        F_A = sym(zeros(3,3));
        F_A(1:3,3) = sym([1 0 0].');
    else
        F_A = zeros(3,3);
        F_A(1:3,3) = [1 0 0].';
    end

    % Extract the parametrization
    z1 = phi(1);
    x  = phi(2);
    z2 = phi(3);
    
    % Compute the full rotation matrix
    C_AB = getRotationMatrixZ(z1)*getRotationMatrixX(x)*getRotationMatrixZ(z2);
    
    % Compute F matrix in origin frame A
    F_A(1:3,1) = [(-cos(x)*sin(z1)/sin(x)); (sin(z1)); (-cos(z1)/sin(x))];
    F_A(1:3,2) = [(cos(x)*cos(z1)/sin(x)); (cos(z1)); (sin(z1)/sin(x))];
    
    % Compute F matrix in origin frame B
    F_B = F_A*C_AB;
    
    if isa(phi,'sym')
        F_A = simplify(F_A);
        F_B = simplify(F_B);
    end
end
