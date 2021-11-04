function A_r = rotVecWithQuat_solution(quat_AB,B_r)
  % Input: the orientation quaternion and the coordinate of the vector to be mapped
  % Output: the coordinates of the vector in the target frame
  C_AB = quatToRotMat_solution(quat_AB);
  A_r = C_AB*B_r;
end
