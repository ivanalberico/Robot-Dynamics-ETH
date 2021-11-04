function [] = forceviz(vizualizer, model, world, t, qdata, varargin)

persistent fvec;
varargin = flatcells(varargin);

% Clear force vectors visuals
if isempty(fvec)
    fvec = cell(2,1);
else
    delete(fvec{1});
    delete(fvec{2});
end

% Extract system data
x = zeros(20,1);
x(1:10) = qdata;
x(11:20) = varargin{1};
F_EE = varargin{2};

% Visualize contact forces
T_ICee = model.body(11).kinematics.compute.T_IBi(x(1:10), x(11:20), [], model.parameters.values);
I_r_ICee = T_ICee(1:3,4);
fvec = genarrowvec(vizualizer.rvaxes, [], 12, 1.0, 'magenta', '$\vec{\bf{F}}_{EE}$', I_r_ICee, F_EE(:));
set(fvec{1},'visible','on');
set(fvec{2},'visible','on');

end