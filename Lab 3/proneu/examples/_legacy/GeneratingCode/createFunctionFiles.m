% createFunctionFiles.m
% -> .m file that explains how to generate different function files:
% * matlab-function
% * mex-functions
% * ccode
%
% This file generates the functions for the robot arm with three links.
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
clc
clear all

load body.mat
load sys.mat
load values.mat

if ~exist('matlabFunc','dir') 
    mkdir matlabFunc
end
if ~exist('mexFunc','dir')
    mkdir mexFunc
end
if ~exist('cFunc','dir')
    mkdir cFunc
end

if isunix
    pathslash = '/';
else
    pathslash = '\';
end

% generate a matlab function
matlabFunction(sys.MpNE,'file',['matlabFunc' pathslash 'Mfunc'],'vars',[sys.q;sys.param]);
matlabFunction(sys.bpNE,'file',['matlabFunc' pathslash 'bfunc'],'vars',[sys.q;sys.dq;sys.param]);
matlabFunction(sys.gpNE,'file', ['matlabFunc' pathslash 'gfunc'],'vars',[sys.q;sys.param]);
% compile this matlab function to a mex function
if isunix
    emlc -o mexFunc/Mfunc_mex matlabFunc/Mfunc.m
    emlc -o mexFunc/bfunc_mex matlabFunc/bfunc.m
    emlc -o mexFunc/gfunc_mex matlabFunc/gfunc.m
else
    emlc -o mexFunc\Mfunc_mex matlabFunc\Mfunc.m
    emlc -o mexFunc\bfunc_mex matlabFunc\bfunc.m
    emlc -o mexFunc\gfunc_mex matlabFunc\gfunc.m
end

% generate c code
% this generates a C-code source file eom_main.c that can be compiled by
% 'g++ eom_main.c -o eom_main -lm' under linux. 
% The code computes MpNE, bpNE, gpNE and fpNE of the EoM based on 
% the parameter values paramDef, generalized coordinate values qDef, dqDef 
% and the actuator forces/torques TDef.
genCCodeExampleFile(['cFunc' pathslash], 'eom_main.c', sys, values);



%% speed check
% this compares the mex function call to the m-file function call.  mex are
% getting significantly faster when the files are large

p = rand(size(sys.param));  % generate random parameters
pc = num2cell(p);
q = rand(size(sys.q));      % generate random generalized coordinates
qc = num2cell(q);
dq = rand(size(sys.dq));      % generate random generalized coordinates
dqc = num2cell(dq);

N = 10000;
cd matlabFunc

tic
for i=1:N
    M = Mfunc(qc{:},pc{:});
    b = bfunc(qc{:},dqc{:},pc{:});
    g = gfunc(qc{:},pc{:});
end
t = toc;
disp(['evaluating m-function took: ', num2str(t/N), 'seconds']);

cd ../mexFunc

tic
for i=1:N
    M = Mfunc_mex(qc{:},pc{:});
    b = bfunc_mex(qc{:},dqc{:},pc{:});
    g = gfunc_mex(qc{:},pc{:});
end
t = toc;
disp(['evaluating mex-function took: ', num2str(t/N), 'seconds']);

cd ..

