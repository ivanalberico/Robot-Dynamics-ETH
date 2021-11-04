function [ q ] = inverseKinematics_solution(I_r_IE_des, C_IE_des, q_0, tol)
% Input: desired end-effector position, desired end-effector orientation (rotation matrix),
%        initial guess for joint angles, threshold for the stopping-criterion
% Output: joint angles which match desired end-effector position and orientation

% 0. Setup
it = 0;
max_it = 100;       % Set the maximum number of iterations. 
lambda = 0.001;     % Damping factor.
alpha = 0.5;        % Update rate

close all;
loadviz;

% 1. start configuration
q = q_0;

% 2. Iterate until terminating condition.
while (it==0 || (norm(dxe)>tol && it < max_it))
    % 3. evaluate Jacobian for current q
    I_J = [jointToPosJac_solution(q); ...
           jointToRotJac_solution(q)];
    
    % 4. Update the psuedo inverse
    I_J_pinv = pseudoInverseMat_solution(I_J, lambda);
    
    % 5. Find the end-effector configuration error vector
    % position error
    I_r_IE = jointToPosition_solution(q);
    dr = I_r_IE_des - I_r_IE; 
    % rotation error
    C_IE = jointToRotMat_solution(q);
    C_err = C_IE_des*C_IE';
    dph = rotMatToRotVec_solution(C_err); 
    % 6D error
    dxe = [dr; dph];
    
    % 6. Update the generalized coordinates
    q = q + alpha*I_J_pinv*dxe;
     
    % Update robot
    abbRobot.setJointPositions(q);
    drawnow;
    pause(0.1);
    
    it = it+1;
end

% Get final error (as for 5.)
% position error
I_r_IE = jointToPosition_solution(q);
dr = I_r_IE_des - I_r_IE; 
% rotation error
C_IE = jointToRotMat_solution(q);
C_err = C_IE_des*C_IE';
dph = rotMatToRotVec_solution(C_err); 

fprintf('Inverse kinematics terminated after %d iterations.\n', it);
fprintf('Position error: %e.\n', norm(dr));
fprintf('Attitude error: %e.\n', norm(dph));
end
