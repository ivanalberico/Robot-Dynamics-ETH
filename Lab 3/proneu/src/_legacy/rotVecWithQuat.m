function A_r = rotVecWithQuat(quat_AB,B_r)
  % Input: the orientation quaternion and the coordinate of the vector to be mapped
  % Output: the coordinates of the vector in the target frame
  C_AB = quatToRotMat(quat_AB);
  A_r = C_AB*B_r;
end
