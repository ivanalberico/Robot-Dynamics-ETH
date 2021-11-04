function T56 = jointToTransform56_solution(q)
  % Input: joint angles
  % Output: homogeneous transformation Matrix from frame 5 to frame 6. T_56
  if (length(q)>1)
  	q = q(6);
  end
  T56 = [1,      0,       0, 0.072;
         0, cos(q), -sin(q),     0;
         0, sin(q),  cos(q),     0;
         0,      0,       0,     1];
end
