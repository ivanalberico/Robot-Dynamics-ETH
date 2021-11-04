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
%   File:           quadrotor_controller.m
%
%   Authors:        Vassilios Tsounis, tsounisv@ethz.ch
%
%   Date:           10/12/2016
%
%   Desciption:     A simple controller for a quadrotor.
%

function [ W_R ] = quadrotor_controller(model,world,logger,t,x)
    
    persistent F_T T_Q;

    % Initialize rotor forces and moments
    if t <= 0.001
        F_T = [0 0 0 0].';
        T_Q = [0 0 0 0].';
    end
    
    % Attidude setpoints
    rpy_desired = [0 0 0].';
    xyz_desired = [1 1 2].';
    
    % Get system measurements
    xyz_measured = x(1:3);
    rpy_measured = x(4:6);
    I_v_measured = x(7:9);
    B_omega_measured = x(10:12);
    
    % Compute TPP velocities
     %V_P = [0 0 0 0].';
    
    % Attitude controller
    omega_R_des = quadrotor_directposition_controller(t, rpy_measured, xyz_measured, B_omega_measured, I_v_measured, rpy_desired(3), xyz_desired);
%     omega_R_des = quadrotor_cascaded_controller(t, rpy_measured, xyz_measured, B_omega_measured, I_v_measured, rpy_desired, xyz_desired);

    % Motor velocity controller
    %V_motors = quadrotor_motor_controllers(t, F_T_desired);
    
    % Compute actuator dynamics step
    %omega_R = quadrotor_motor_dynamics(t, V_motors, T_Q);
    
    % Compute thrust forces and drag moments
    %[ F_T, T_Q ] = quadrotor_rotor_aerodynamics(t, omega_R, V_P);
    
    % Compute final output
    % Model params
    b = 8.5e-6;
    d = 8.04e-5;
    F_T = [b b b b].' .* omega_R_des.^2;
    T_Q = [-d d -d d].' .* omega_R_des.^2;
    
    % Set rotor wrench parameters
    W_R = [F_T; T_Q];
    
end

%
% SUBSYSTEM FUNCTIONS
%

function [ omega_R_des ] = quadrotor_cascaded_controller(t, rpy, xyz, B_omega, I_v, rpy_des, xyz_des)

    % Declare controller integrators
    persistent integrators_rpy integrator_z;
    
    % Initialize integrators
    if t <= 0.001
        integrators_rpy = zeros(3,1);
        integrator_z = 0;
    end
    
    % Attitude controller
    att_Kp = [10; 10; 5];
    att_Kd = [8; 8; 3]; 
    att_Ki = [0; 0; 0];
    
    % Altitude controller
    alt_Kp = 25.0;
    alt_Kd = 15.0;
    alt_Ki = 0;
    
    % Model params
    l = 0.4;
    b = 8.5e-6;
    d = 8.04e-5;
    omega_max = 5236; % rad/s for MAX_RPM = 50,000.0
    m = 2.14;
    g = 9.18;
    
    % Compute Control Allocation mapping
    N = [1/(4*b)  0            1/(2*l*b)   -1/(4*d);
         1/(4*b)  -1/(2*l*b)   0           1/(4*d);
         1/(4*b)  0            -1/(2*l*b)  -1/(4*d);
         1/(4*b)  1/(2*l*b)    0           1/(4*d)];
    
    % Initialize virtual controller
    U = zeros(4,1);
    
    % Get rotation rate map
    [F_A, F_B] = getInverseRotationRateMapCardanXyz(rpy);
    dphi_B = F_B * B_omega;
    
    % Attitude controller
    atterr = rpy_des - rpy;
    angvelerr = -dphi_B;
    integrators_rpy = integrators_rpy + atterr;
    U(2:4) = att_Kp.*atterr + att_Kd.*angvelerr + att_Ki.*integrators_rpy;
    
    % Altitude Controller
    z_poserr = xyz_des(3) - xyz(3);
    z_velerr = -I_v(3);
    integrator_z = integrator_z + z_poserr;
    T_z = alt_Kp*z_poserr + alt_Kd*z_velerr + alt_Ki*integrator_z - m*g;
    U(1) = - T_z / (cos(rpy(1))*cos(rpy(2)));
    
    % Sign corrections
    U(1) = -U(1);
    U(3) = - U(3);
    
    % Control Allocation
    omega_sqr = N*U;
    
    % Saturate for negative squared velocity and maximum positive
    for i=numel(omega_sqr)
        if omega_sqr(i) < 0
            omega_sqr(i) = 0;
        elseif omega_sqr(i) > (omega_max^2)
            omega_sqr = (omega_max^2);
        end
    end
    
    % Compute final output
    omega_R_des = sqrt(omega_sqr);
    
end

function [ omega_R_des ] = quadrotor_directposition_controller(t, rpy, xyz, B_omega, I_v, yaw_des, xyz_des)

    % Declare controller integrators
    persistent integrators_xyz integrators_rpy;
    
    % Initialize integrators
    if t <= 0.001
        integrators_xyz = zeros(3,1);
        integrators_rpy = zeros(3,1);
    end
    
    % Position controller
    pos_Kp = [8; 8; 25];
    pos_Kd = [15; 15; 15]; 
    pos_Ki = [0.005; 0.005; 0.005];
    
    % Attitude controller
    att_Kp = [10; 10; 5];
    att_Kd = [8; 8; 3]; 
    att_Ki = [0; 0; 0];
    
    % Model params
    l = 0.4;
    b = 8.5e-6;
    d = 8.04e-5;
    omega_max = 5236; % rad/s for MAX_RPM = 50,000.0
    
    % Compute Control Allocation mapping
    N = [1/(4*b)  0            1/(2*l*b)   -1/(4*d);
         1/(4*b)  -1/(2*l*b)   0           1/(4*d);
         1/(4*b)  0            -1/(2*l*b)  -1/(4*d);
         1/(4*b)  1/(2*l*b)    0           1/(4*d)];
    
    % Compute desired inertial thrust
    poserr = xyz_des - xyz;
    velerr = -I_v;
    integrators_xyz = integrators_xyz + poserr;
    I_F_thr = pos_Kp.*poserr + pos_Kd.*velerr + pos_Ki.*integrators_xyz;
    F_thr_des = norm(I_F_thr,2);
    
    % Compute vector of thrusts in intermediate frame
    X = (1/F_thr_des) * getRotationMatrixZ(rpy(3)).' * I_F_thr;
    
    % Compute desired attitude
    phi_des = asin(-X(2));
    theta_des = atan2(X(1),X(3));
    psi_des = yaw_des;
    
    % Attitude controller
    att_des = [phi_des theta_des psi_des].';
    atterr = att_des - rpy;
    angvelerr = -B_omega;
    integrators_rpy = integrators_rpy + atterr;
    T_att_des = att_Kp.*atterr + att_Kd.*angvelerr + att_Ki.*integrators_rpy;
    
    % Set virtual controller values
    U = [F_thr_des; T_att_des];
    
    % Sign corrections
    U(3) = -U(3);
    
    % Control Allocation
    omega_sqr = N*U;
    
    % Saturate for negative squared velocity and maximum positive
    for i=numel(omega_sqr)
        if omega_sqr(i) < 0
            omega_sqr(i) = 0;
        elseif omega_sqr(i) > (omega_max^2)
            omega_sqr = (omega_max^2);
        end
    end
    
    % Compute final output
    omega_R_des = sqrt(omega_sqr);
    
end

function [ omega_R_desired ] = quadrotor_motor_controllers(t, F_T_desired)

    % Declare controller integrators
    persistent integrators;
    
    % Initialize integrators
    if t <= 0.001
        integrators = zeros(4,1);
    end
    
    %
    % TODO
    %

    % Set desired motor velocities
    omega_R_desired = [0 0 0 0].';
    
end

function [ omega_R, P_Rm, P_Re ] = quadrotor_motor_dynamics(t, V_m, T_Q)
    
    % Declare motor speeds
    persistent omega_m i_m;

    % Initialize integrators
    if t <= 0.001
        omega_m = zeros(4,1);
        i_m = zeros(4,1);
    end
    
    % MOTOR PARAMETERS
    L_m = 1e-7*ones(4,1);
    R_m = 0.3*ones(4,1);
    K_T = 0.0015*ones(4,1);
    J_m = 1.0e-5*ones(4,1);
    DeltaT = 5e-3; % WARNING THIS IS TOO SLOW FOR MOTOR DYNAMICS
    
    % Compute Euler-Backward update of the motor current and speed
    i_m = i_m + (DeltaT/L_m) * (V_m - K_T*omega_m - R_m*i_m);
    omega_m = omega_m + (DeltaT/J_m) * (K_T*i_m - T_Q);
    
    % Compute total power
    P_Rm = K_T*i_m*omega_m;
    P_Re = i_m*V_m;
    
    % Set vector of motor speeds
    omega_R = omega_m;

end

function [ F_T, T_Q ] = quadrotor_rotor_aerodynamics(t, omega_R, V_P)

    %
    % NOTE: we use only the simplified model here
    %

    % Set constants
    B = 8.5e-6; %thrust constant
    D = 8.04e-5; %drag constant
    
    % Compute forces and moments
    F_T = B * omega_R.^2;
    T_Q = D * omega_R.^2;

end

    