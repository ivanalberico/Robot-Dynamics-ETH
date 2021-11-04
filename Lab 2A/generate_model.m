%% Define Generalized Coordintates & Model Parameters

% Initialize generalized coordinates
gen_cor = generate_gen_cor;

% Model parameters
params = generate_params;

%% Generate Forward Kinematics

kin = generate_kin(gen_cor);

%% Generate Forward Differential Kinematics

jac = generate_jac(gen_cor, kin, params);

%% Generate Equations of Motion

eom = generate_eom_solution(gen_cor, kin, params, jac);

%%
% EOF