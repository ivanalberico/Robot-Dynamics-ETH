function R = eulAngXyzToRotMat(angles)
% EULANGXYZTOROTMAT(angles) maps XYZ Euler angles to a rotation matrix.

x = angles(1);
y = angles(2);
z = angles(3);

rotx = @(q) [1, 0, 0; 0, cos(q), -sin(q); 0 sin(q) cos(q)];
roty = @(q) [cos(q), 0, sin(q); 0, 1, 0; -sin(q), 0, cos(q)];
rotz = @(q) [cos(q), -sin(q), 0; sin(q), cos(q), 0; 0, 0, 1];

R = rotx(x)*roty(y)*rotz(z);

end