function quat_inv = invertQuat(quat)
  % Input: a unit quaternion
  % Output: the inverse of the input quaternion
  quat = quat(:);
  quat_inv = [quat(1); -quat(2:end)];
  quat_inv = quat_inv/norm(quat_inv);
end