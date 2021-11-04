function T12 = jointToTransform12_solution(q)
  % Input: joint angles
  % Output: homogeneous transformation Matrix from frame 2 to frame 1. T_12
  if (length(q)>1)
  	q = q(2);
  end
  T12 = [ cos(q), 0, sin(q),     0;
               0, 1,      0,     0;
         -sin(q), 0, cos(q), 0.145;
               0, 0,      0,     1];
     
end

