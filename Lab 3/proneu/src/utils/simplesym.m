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

function [ S ] = simplesym(S, mode, ns)

    % Simplify if S is a symbolic expression and the steps ns is nonzero
    if isa(S, 'sym') && (ns ~= 0)
    
        switch mode
            
            case 'serial'
                S = simplify(S, ns);
                
            case 'parallel'
                S = parsimplify(S, ns);
                
            case 'elementwise'
                S = elemsimplify(S, ns);
                
            otherwise
                warning('Incorrect mode for symbolic simplification. No simplifcations were performed.');
        end
    end
end

function [ S ] = parsimplify(S, ns)

    % Get dimensions of the symbolic expression
    [ Ni, Nj ] = size(S);
    
    % Set total number of elements
    Ne = Ni * Nj;
    
    % Pre-allocate temporary array
    s = sym(zeros(Ne,1));
    
    % Copy data into temporary s
    for n=1:Ne
        i = mod(n-1,Ni)+1;
        j = (n - i)/Ni + 1;
        s(n) = S(i, j);
    end 
    
    % Compute the intrinsic body dynamics
    parfor (n=1:Ne, getpoolnum())
        % Simplify array element
        s(n) = simplify(s(n), ns);
    end
    
    % Copy data back into S
    for n=1:Ne
        i = mod(n-1,Ni)+1;
        j = (n - i)/Ni + 1;
        S(i, j) = s(n);
    end 
end

function [ S ] = elemsimplify(S, ns)

    % Perform element-wise implifications
    for i=1:numel(S)
        S(i) = simplify(S(i), ns);
    end
    
end


