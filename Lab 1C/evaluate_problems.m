% This script allows you to test your implementation in ./problems against
% the encripted solution files in ./solutions/pcodes

% Test settings
N = 1000;   % number of tests
tol = 1e-6; % test tolerance

% --- Convenience functions ---
almostequal = @(x1, x2) all(abs(x1 - x2) < tol);
disp_correct = @(fun) fprintf('%s: correct \n', fun);
disp_incorrect = @(fun) fprintf('%s: incorrect \n', fun);

rotx = @(q) [1, 0, 0; 0, cos(q), -sin(q); 0 sin(q) cos(q)];
roty = @(q) [cos(q), 0, sin(q); 0, 1, 0; -sin(q), 0, cos(q)];
rotz = @(q) [cos(q), -sin(q), 0; sin(q), cos(q), 0; 0, 0, 1];
get_rand_rot = @() rotx(randn(1))*roty(randn(1))*rotz(randn(1));

% -----------------------------

fprintf('Evaluating user implementations vs. solutions... \n')

% Evaluate pseudoInverseMat.m
fun = 'pseudoInverseMat';
correct = 0;
for i = 1:N
    % Make random matrix with random dimensions
    a = ceil(10*rand(1));
    b = ceil(10*rand(1));
    A = rand(a,b);
    lambda = 1e-3*rand(1);
    user_output = feval(fun, A, lambda);
    solution = feval([fun, '_solution'], A, lambda);
    if almostequal(user_output, solution)
        correct = correct + 1;
    end
end
if correct == N
    disp_correct(fun);
else
    disp_incorrect(fun);
end

% Evaluate rotMatToRotVec.m
fun = 'rotMatToRotVec';
correct = 0;
for i = 1:N
    C = get_rand_rot();
    user_output = feval(fun, C);
    solution = feval([fun, '_solution'], C);
    if almostequal(user_output, solution)
        correct = correct + 1;
    end
end
if correct == N
    disp_correct(fun);
else
    disp_incorrect(fun);
end
fprintf('... Done \n')

fprintf('\nFor the other functions: \n')
fprintf('- inverseKinematics: Final pose error given at the end of the function \n');
fprintf('- kinematicMotionControl: Use motion_control_visualization.m with "use_solution = 1" \n');



