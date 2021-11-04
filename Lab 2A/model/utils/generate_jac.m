% generate jacobians
function jac = generate_jac(gen_cor, kin, params)
% By calling:
%   jac = generate_jac(gen_cor, kin, dyn)
% a struct 'jac' is returned that contains the translation and rotation
% jacobians of the center of masses

%% Setup
phi = gen_cor.phi;
dphi = gen_cor.dphi;

T_Ik = kin.T_Ik;
R_Ik = kin.R_Ik;
k_n_k = kin.k_n_k;
I_r_Ie = kin.I_r_Ie;

k_r_ks = params.k_r_ks;

jac.I_Jp_s = cell(6,1);
jac.I_Jr = cell(6,1);
jac.I_Jpe = sym(zeros(3,6));
jac.I_Jre = sym(zeros(3,6));


%% Compute link jacobians

I_Jp_s = cell(6,1);
I_Jr = cell(6,1);

for k=1:6
    
    % create containers
    I_Jp_s{k} = sym(zeros(3,6));
    I_Jr{k} = sym(zeros(3,6));
    
    % translational jacobian at the center of gravity s in frame I
    I_r_ks = [eye(3) zeros(3,1)]*T_Ik{k}*[k_r_ks{k};1];
    I_Jp_s{k} = jacobian(I_r_ks,phi);
    
    % rotational jacobian in frame I
    if k == 1
        I_Jr{1}(1:3,1) = R_Ik{1} * k_n_k{1};
    else
        % copy columns of k-1 jacobian
        I_Jr{k} = I_Jr{k-1};
        
        % evaluate new column
        I_Jr{k}(1:3,k) = R_Ik{k} * k_n_k{k};
    end

    % simplify expressions
    I_Jp_s{k} = simplify(I_Jp_s{k});
    I_Jr{k} = simplify(I_Jr{k});
    
end

% Compute the end effector jacobians in frame I
I_Jpe = simplify(jacobian(I_r_Ie,phi));
I_Jre = I_Jr{6};

% Compute the time derivative of the end effector Jacobians
I_dJpe = simplify(dAdt(I_Jpe,phi,dphi));
I_dJre = simplify(dAdt(I_Jre,phi,dphi));

I_Je = [I_Jpe; I_Jre];
I_dJe = [I_dJpe; I_dJre];

% Generate function files from symbolic expressions
fname = mfilename;
fpath = mfilename('fullpath');
dpath = strrep(fpath, fname, '');
dpath = strcat(dpath,'../irb120/');
fprintf('Generating Jacobian file... ');
matlabFunction(I_Je, 'vars', {phi}, 'file', strcat(dpath,'/I_Je_fun'));
fprintf('done!\n')
fprintf('Generating dJe file... ');
matlabFunction(I_dJe, 'vars', {phi,dphi}, 'file', strcat(dpath,'/I_dJe_fun'));
fprintf('done!\n')

% Store jacobians in output struct
jac.I_Jp_s = I_Jp_s;
jac.I_Jr = I_Jr;
jac.I_Jpe = I_Jpe;
jac.I_Jre = I_Jre;
jac.I_dJpe = I_dJpe;
jac.I_dJre = I_dJre;


end