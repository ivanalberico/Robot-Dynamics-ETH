% generate dynamic parameters
function params = generate_params()
%% Create dynamics container

% CoM of link k with respect to frame k
params.k_r_ks = cell(6,1);

% Mass of link k
params.m = cell(6,1);

% Inertia tensor of link k with respect to frame k
params.k_I_s = cell(6,1);

% Gravity acceleration with respect to the inertial frame
params.I_g_acc = zeros(3,1);


%% Link properties

% Link 1
r_1s = [0; 0; 0.062];
m_1 = 3.0;
I_1 = [0.014 0 0;
       0 0.010 0;
       0 0 0.014];

% Link 2
r_2s = [0; 0; 0.12];
m_2 = 3.9;
I_2 = [0.060 0 0;
       0 0.026 0;
       0 0 0.042];

% Link 3
r_3s = [0.5; 0; 0.03];
m_3 = 2.9;
I_3 = [0.008 0 0;
       0 0.013 0;
       0 0 0.017];

% Link 4
r_4s = [0; 0; 0.07];
m_4 = 1.3;
I_4 = [0.003 0 0;
       0 0.005 0;
       0 0 0.004];

% Link 5
r_5s = [0; 0; 0.03];
m_5 = 0.55;
I_5 = [0.0004 0 0;
       0 0.0008 0;
       0 0 0.0009];

% Link 6
r_6s = [0; 0; 0];
m_6 = 0.014;
% I_6 = [0.000002 0 0;
%        0 0.000002 0;
%        0 0 0.000003];
I_6 = [0.002 0 0;
       0 0.002 0;
       0 0 0.003];


params.k_r_ks{1} = r_1s;
params.m{1} = m_1;
params.k_I_s{1} = I_1;

params.k_r_ks{2} = r_2s;
params.m{2} = m_2;
params.k_I_s{2} = I_2;

params.k_r_ks{3} = r_3s;
params.m{3} = m_3;
params.k_I_s{3} = I_3;

params.k_r_ks{4} = r_4s;
params.m{4} = m_4;
params.k_I_s{4} = I_4;

params.k_r_ks{5} = r_5s;
params.m{5} = m_5;
params.k_I_s{5} = I_5;

params.k_r_ks{6} = r_6s;
params.m{6} = m_6;
params.k_I_s{6} = I_6;


%% Gravity
params.I_g_acc = [0; 0; -9.81];
    
end