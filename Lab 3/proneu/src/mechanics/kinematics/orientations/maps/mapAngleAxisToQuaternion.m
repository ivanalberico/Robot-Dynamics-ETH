function q = mapAngleAxisToQuaternion(theta, n)
% MAPANGLEAXISTOQUATERNION(th, r) maps an angle-axis parametrization to a
% quaternion one, represented as q = [qw qx qy qz]'
%
% Author(s): Dario Bellicoso

q0 = cos(theta/2);
qv = sin(theta/2)*n(:);
q = [q0; qv];

end
