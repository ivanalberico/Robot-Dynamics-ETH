function T23 = jointToTransform23_solution(q)
  % Input: joint angles
  % Output: homogeneous transformation Matrix from frame 2 to frame 3. T_23
  if (length(q)>1)
  	q = q(3);
  end
  T23 = [ cos(q), 0, sin(q),     0;
               0, 1,      0,     0;
         -sin(q), 0, cos(q), 0.270;
               0, 0,      0,     1];
end
