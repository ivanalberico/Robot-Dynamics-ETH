function quat_AC = quatMult(quat_AB,quat_BC)
  % Input: two quaternions to be multiplied
  % Output: output of the multiplication
  q = quat_AB;
  p = quat_BC;
  
  q_w = q(1); q_n = q(2:4);
  p_w = p(1); p_n = p(2:4);
  
  skew_q_n = [0      -q_n(3)  q_n(2);
              q_n(3)  0      -q_n(1);
             -q_n(2)  q_n(1)  0];

  quat_AC = [q_w*p_w - q_n'*p_n;
             q_w*p_n + p_w*q_n + skew_q_n*p_n];
end