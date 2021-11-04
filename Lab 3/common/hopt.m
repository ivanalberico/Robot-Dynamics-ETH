function x = hopt(A, b, C, d, print_solution)
% Solves the hierarchical optimization problem with for each task
% while preserving optimality of higher order tasks
% 
%            min_{x} 0.5*||A*x - b||^2
%            s.t. C*x <= d

% Initialize
num_tasks = length(A);
num_constraints = length(C);
Aineq = C;
bineq = d;
Aeq = [];
beq = [];
x = [];

optionsQP.Display = 'off';
optionsQP.Algorithm = 'interior-point-convex';

for opt_iter = 1:num_tasks
    % Current task
    Ai = A{opt_iter};
    bi = b{opt_iter};
    
    if isempty(Ai) % Empty Ai -> No costfunction
        if print_solution
            warning('No cost function specified for task %i', opt_iter)
        end
        continue;
    end
    % Setup Quadratic program
    % Solving min 0.5*||Ax - b||_2 = min 0.5* x'*(A'*A)*x - b'*A*x
    H = Ai'*Ai;
    f = -Ai'*bi;
    [x_new,~,exitflag] = quadprog(H, f, Aineq, bineq, Aeq, beq, [], [], [], optionsQP);
    
    if(exitflag ~= 1)
        if print_solution
            warning('Optimization did not succeed for iteration %i',opt_iter)
        end
        break
    else
        x = x_new; % update with new solution
        % Append minimization to equality constraints
        Aeq = [Aeq; Ai];
        beq = [beq; Ai*x];
    end
end

if print_solution
    for t = 1:opt_iter
        fprintf('==== Task %d ====\n', t);
        fprintf('||Ax - b||^2 = %f\n', norm(A{t}*x - b{t})^2);
%         if(t <= num_constraints)
            for ii = 1:size(C,1)
                fprintf('%d C*x - d = %f\n', ii, C(ii,:)*x - d(ii));
            end
%         end
    end
end

end