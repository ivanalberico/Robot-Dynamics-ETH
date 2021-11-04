% function schar=genCCodeMatrix(filename, m, arrayname)
%
% -> generates C-Code from the symbolic matrix m. 
% The code is temporarily stored in a file.
% 
% INPUTS:
%   path         path to C-code temporary file (add trailing /)
%   filename     name of the temporary file
%   m            symbolic matrix
%   arrayname    name of the C-code array
%
% OUTPUTS:
%   schar   string containing the C-Code
%
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
function schar=genCCodeMatrix(path, filename, m, arrayname)

% create c-code in file
ccode(m, 'file',[path filename]);

% open file and read code
fid = fopen([path filename],'r');
scell = textscan(fid,'%s','Delimiter','\n', 'BufSize', 70000);
fclose(fid);
schar = scell{1};


% add definitions to auxiliary variables
for k=1:length(schar)
    schar{k} = ['const double ' schar{k}];
end

% replace name of array
schar = strrep(schar, 'const double MatrixWithNoName', arrayname);
schar = strrep(schar, 'MatrixWithNoName', arrayname);

% get rid of variable t0 that is not used
schar = strrep(schar, 'const double t0 = ', '');

% rewrite file
fid = fopen([path filename],'w');
for k=1:length(schar)
    fprintf(fid,'%s\n',schar{k});
end
fclose(fid);