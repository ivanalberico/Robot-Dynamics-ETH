function q = inverseKinematics(I_r_IE_des, C_IE_des, q_0, tol)
  % Input: desired end-effector position, desired end-effector orientation (rotation matrix), 
  %        initial guess for joint angles, threshold for the stopping-criterion
  % Output: joint angles which match desired end-effector position and orientation
  
  %% Setup
  it = 0;
  I_r_IE_des = I_r_IE_des(:);
  q_0 = q_0(:);
  
  % Set the maximum number of iterations.
  max_it = 1000;

  % Initialize the solution with the initial guess.
  q = q_0;
  
  % Damping factor.
  lambda = 0.001;

  % Initialize error.
  C_IE = jointToRotMat(q);
  I_r_IE = jointToPosition(q);
  C_err = C_IE_des*C_IE';
  dph = rotMatToRotVec(C_err);
  dr = I_r_IE_des - I_r_IE;
  dxe = [dr; dph];
  
  close all;
  loadVisualization;
  
  
  %% Iterative inverse kinematics
  
  % Iterate until terminating condition.
  while (norm(dxe)>tol && it < max_it)

    I_J = [jointToPosJac(q);
    	   jointToRotJac(q)];

    dq = pseudoInverseMat(I_J, lambda)*[dr;
                                        dph];
    
    % Update law.
    q = q + 0.5*dq;
    
    % Update error
    C_IE = jointToRotMat(q);
    C_err = C_IE_des*C_IE';
    dph = rotMatToRotVec(C_err);
    
    dr = I_r_IE_des - jointToPosition(q);
    dxe = [dr; dph];
    
    abbRobot.setJointPositions(q);
    drawnow;
    pause(0.1);
    
    it = it+1;
  end
  
  fprintf('Inverse kinematics terminated after %d iterations.\n',it);
  fprintf('Position error: %e.\n',norm(dr));
  fprintf('Attitude error: %e.\n',norm(dph));
  
end
