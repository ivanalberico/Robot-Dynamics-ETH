function quat = rotMatToQuat(C)
  % Input: rotation matrix
  % Output: corresponding quaternion [w x y z]
  quat = 0.5*[sqrt((1+trace(C)));
              sign(C(3,2)-C(2,3)) * sqrt(C(1,1) - C(2,2) - C(3,3) + 1);
              sign(C(1,3)-C(3,1)) * sqrt(C(2,2) - C(3,3) - C(1,1) + 1);
              sign(C(2,1)-C(1,2)) * sqrt(C(3,3) - C(1,1) - C(2,2) + 1)];
end