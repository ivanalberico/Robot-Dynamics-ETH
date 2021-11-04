function f_EE = wall(model, x, x_wall)
% wall is in x-plane with offset x_wall

q = x(1:10);   %[10x1] Generalized coordinates
dq = x(11:20); % [10X1] Generalized velocities

params = model.parameters.values;
I_J_EE = eval_jac(model.body(11).kinematics.compute.I_J_IBi, q, [], params);	% [3x10] Arm End-effector position and orientation Jacobian
I_T_EE = model.body(11).kinematics.compute.T_IBi(q, dq, [], params);
x_EE = I_T_EE(1, 4);
v_EE = I_J_EE(1,:)*dq;

K = 1e3;
D = 2*sqrt(K);

if x_EE > x_wall
    f_EE = [K*(x_wall-x_EE) - D*v_EE; 0; 0];
else
    f_EE = zeros(3,1);
end

end

function jac = eval_jac(f, q, dq, params)
jac = f(q,dq,[],params);
jac = jac(1:2:end,:);
end
