%% Generalized coordinates
function gen_cor = generate_gen_cor()

    % generate generalized coordinate symbolic variables
    syms phi1 phi2 phi3 phi4 phi5 phi6 'real';
    syms dphi1 dphi2 dphi3 dphi4 dphi5 dphi6 'real';

    % Define minimal/generalized coordinate vectors
    phi = [phi1 phi2 phi3 phi4 phi5 phi6]';
    dphi = [dphi1 dphi2 dphi3 dphi4 dphi5 dphi6]';

    % Return struct with generalized coordinates and velocities
    gen_cor.phi = phi;
    gen_cor.dphi = dphi;

end