% function schar=genCCodeVariables(var, varDef)
%
% -> generates the definition of a variable and assigns a value.
% For instance var=sym(phi), varDef=0.1 -> const double phi = 0.1;
% 
% INPUTS:
%   var     symbolic vector of the variables
%   varDef  vector of values of the variables
%
% OUTPUTS:
%   schar   string containing the C-Code
%
% proNEu: tool for symbolic EoM derivation
% Copyright (C) 2011  Marco Hutter, Christian Gehring
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
function schar=genCCodeVariables(var, varDef)

for k=1:length(var)
    schar{k} =  ['const double ' char(var(k)) ' = ' num2str(varDef(k)) ';'];
end
