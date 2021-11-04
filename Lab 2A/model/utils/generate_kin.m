% generate kinematics
function kin = generate_kin(gen_cor)
%% Setup

phi = gen_cor.phi;


%% Create kinematics container

kin = struct();

% homogeneous transformations from frame k to frame k-1 = j
kin.T_jk = cell(6,1);

% homogeneous transformations from frame k to the inertial frame
kin.T_Ik = cell(6,1);

% rotation matrices from frame k to the inertial frame
kin.R_Ik = cell(6,1);

% rotation axis of the DoF k with respect to frame k
kin.k_omega_hat_k = cell(6,1);

% homogeneous transformation matrix from the end-effector frame to the
% inertial frame
kin.T_Ie = sym(zeros(4,4));

% position of the end-effector expressed in the inertial frame
kin.I_r_Ie = sym(zeros(3,1));


%% Homogeneous transformations

% homogeneous transformations from frame k to frame k-1 = j
kin.T_jk{1} = jointToTransform01_solution(phi);
kin.T_jk{2} = jointToTransform12_solution(phi);
kin.T_jk{3} = jointToTransform23_solution(phi);
kin.T_jk{4} = jointToTransform34_solution(phi);
kin.T_jk{5} = jointToTransform45_solution(phi);
kin.T_jk{6} = jointToTransform56_solution(phi);

% homogeneous transformations from frame k to frame I
kin.T_Ik{1} = kin.T_jk{1};
kin.T_Ik{2} = simplify(kin.T_Ik{1}*kin.T_jk{2});
kin.T_Ik{3} = simplify(kin.T_Ik{2}*kin.T_jk{3});
kin.T_Ik{4} = simplify(kin.T_Ik{3}*kin.T_jk{4});
kin.T_Ik{5} = simplify(kin.T_Ik{4}*kin.T_jk{5});
kin.T_Ik{6} = simplify(kin.T_Ik{5}*kin.T_jk{6});

% rotation matrices from frame k to frame I
kin.R_Ik{1} = kin.T_Ik{1}(1:3,1:3);
kin.R_Ik{2} = kin.T_Ik{2}(1:3,1:3);
kin.R_Ik{3} = kin.T_Ik{3}(1:3,1:3);
kin.R_Ik{4} = kin.T_Ik{4}(1:3,1:3);
kin.R_Ik{5} = kin.T_Ik{5}(1:3,1:3);
kin.R_Ik{6} = kin.T_Ik{6}(1:3,1:3);

% joint rotation axes in the k frame
kin.k_n_k{1} = [0 0 1]';
kin.k_n_k{2} = [0 1 0]';
kin.k_n_k{3} = [0 1 0]';
kin.k_n_k{4} = [1 0 0]';
kin.k_n_k{5} = [0 1 0]';
kin.k_n_k{6} = [1 0 0]';


%% Endeffector

% end-effector homogeneous transformation and position
kin.T_Ie = simplify(kin.T_Ik{6});
kin.I_r_Ie = kin.T_Ie(1:3,4);

%% Matalb functions

fname = mfilename;
fpath = mfilename('fullpath');
dpath = strrep(fpath, fname, '');
dpath = strcat(dpath,'../irb120/');

fprintf('Generating end-effector position file... ');
matlabFunction(kin.I_r_Ie, 'vars', {phi}, 'file', strcat(dpath,'/I_r_IE_fun'));
fprintf('done!\n')

fprintf('Generating forward kinematics file... ');
matlabFunction(kin.T_Ie, 'vars', {phi}, 'file', strcat(dpath,'/T_IE_fun'));
fprintf('done!\n')

end
