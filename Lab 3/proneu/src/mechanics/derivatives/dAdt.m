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
% function dA = dAdt ( A, x, dx )
%
% This function computes the total time derivation of a matrix or array A,
% which is a function of another vector x. The derivative is computed via
% the chain rule:
%
% dA(x) = dA(x)/dx * dx, 
% 
% Where the element dx representes the time derivative dx/dt, and thus
% dA(x) then represents the time derivative dA(x)/dt.

function [ dA ] = dAdt( A, x, dx )
    
    % Initialize the derivative to zeros 
    dA = sym(zeros(size(A)));

    % Compute the element-wise partial derivatives as Jacobians
    for i=1:size(A,1)
        for j=1:size(A,2)
            dA(i,j) = jacobian(A(i,j), x) * dx;
        end
    end
end

