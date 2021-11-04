% This script allows you to test your implementation in ./problems against
% the encripted solution files in ./solutions/pcodes

% Test settings
N = 1000;   % number of tests
tol = 1e-6; % test tolerance

% --- Convenience functions ---
almostequal = @(x1, x2) all(abs(x1 - x2) < tol);
disp_correct = @(fun) fprintf('%s: correct \n', fun);
disp_incorrect = @(fun) fprintf('%s: incorrect \n', fun);

% -----------------------------

fprintf('Evaluating user implementations vs. solutions... \n')

% Evaluate functions with q as input
for fun_cell = {'jointToPosJac', ...
                'jointToRotJac'}
    fun = fun_cell{:}; % Read string from cell
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

fprintf('... Done \n')

