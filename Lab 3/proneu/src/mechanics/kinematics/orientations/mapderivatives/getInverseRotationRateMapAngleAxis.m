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

function [ F_A, F_B ] = getInverseRotationRateMapAngleAxis(theta, n)
    
    % Intilize rate matrix
    if isa(theta,'sym') || isa(n,'sym')
        F_A = sym(zeros(4,3));
    else
        F_A = zeros(4,3);
    end

    % Ensure n is a column vector
    n = n(:);
    
    % Compute the full rotation matrix
    C_AB = mapAngleAxisToRotationMatrix(theta, n);
    
    % Compute F matrix
    F_A(1,1:3) = n.';
    F_A(2:4,1:3) = (-1/2)*(sin(theta)/(1-cos(theta)))*skew(n)*skew(n)-(1/2)*skew(n);
    
    % Compute F matrix in origin frame B
    F_B = F_A*C_AB;
    
    if isa(theta,'sym') || isa(n,'sym')
        F_A = simplify(F_A);
        F_B = simplify(F_B);
    end
    
end
