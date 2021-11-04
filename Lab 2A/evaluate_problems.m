% This script allows you to test your implementation in ./problems against
% the encripted solution files in ./solutions/pcodes

% Test settings
N = 100;   % number of tests
tol = 1e-6; % test tolerance

% --- Convenience functions ---
almostequal = @(x1, x2) all(abs(x1 - x2) < tol);
disp_correct = @(fun) fprintf('%s: correct \n', fun);
disp_incorrect = @(fun) fprintf('%s: incorrect \n', fun);
% -----------------------------

fprintf('Evaluating user implementations vs. solutions... \n')

% Evaluate M_fun.m, g_fun
for funcell = {'M_fun', 'g_fun'}
    fun = funcell{1};
    correct = 0;
    for i = 1:N
        q = randn(6,1);
        user_output = feval(fun, q);
        solution = feval([fun, '_solution'], q);
        if almostequal(user_output, solution)
            correct = correct + 1;
        end
    end
    if correct == N
        disp_correct(fun);
    else
        disp_incorrect(fun);
    end
end

% Evaluate b_fun.m, hamiltonian_fun
for funcell = {'b_fun', 'hamiltonian_fun'}
    fun = funcell{1};
    correct = 0;
    for i = 1:N
        q = randn(6,1);
        qd = randn(6,1);
        user_output = feval(fun, q, qd);
        solution = feval([fun, '_solution'], q, qd);
        if almostequal(user_output, solution)
            correct = correct + 1;
        end
    end
    if correct == N
        disp_correct(fun);
    else
        disp_incorrect(fun);
    end
end

fprintf('... Done \n')




