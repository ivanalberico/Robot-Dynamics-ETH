function ddphi = abb_eom_solution(tau, phi, dphi, friction, enable_g, enable_friction)

% Get the joint accelerations by solving the forward dynamics.
ddphi  = M_fun_solution(phi)\(tau - (b_fun_solution(phi,dphi) ...
                         + enable_friction*diag(friction)*dphi ...
                         + enable_g*g_fun_solution(phi)));

end
