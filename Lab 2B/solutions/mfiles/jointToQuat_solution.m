function quat = jointToQuat_solution(q)
  % Input: joint angles
  % Output: quaternion representing the orientation of the end-effector 
  % q_IE.
  C = jointToRotMat_solution(q);
  quat = rotMatToQuat_solution(C);
end

