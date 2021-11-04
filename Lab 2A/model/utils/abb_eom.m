function ddphi = abb_eom(tau, phi, dphi, friction, enable_g, enable_friction)

% Get the joint accelerations by solving the forward dynamics.
ddphi  = M_fun(phi)\(tau - (b_fun(phi,dphi) ...
                         + enable_friction*diag(friction)*dphi ...
                         + enable_g*g_fun(phi)));

end