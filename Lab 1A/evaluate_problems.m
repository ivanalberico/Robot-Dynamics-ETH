% This script allows you to test your implementation in ./problems against
% the encripted solution files in ./solutions/pcodes

% Test settings
N = 1000;   % number of tests
tol = 1e-6; % test tolerance

% --- Convenience functions ---
almostequal = @(x1, x2) all(abs(x1 - x2) < tol);
disp_evaluation = @(fun, correct, N) fprintf('Tested %s: correct %d/%d \n', fun, correct, N);
norm_quat = @(q) q/norm(q);
get_rand_quat = @() norm_quat(randn(4,1));

rotx = @(q) [1, 0, 0; 0, cos(q), -sin(q); 0 sin(q) cos(q)];
roty = @(q) [cos(q), 0, sin(q); 0, 1, 0; -sin(q), 0, cos(q)];
rotz = @(q) [cos(q), -sin(q), 0; sin(q), cos(q), 0; 0, 0, 1];
get_rand_rot = @() rotx(randn(1))*roty(randn(1))*rotz(randn(1));
% -----------------------------

fprintf('Evaluating user implementations vs. solutions for %d random inputs... \n', N)

% Evaluate functions without inputs
for fun_cell = {'getTransformI0', ...
                'getTransform6E'}
    fun = fun_cell{:}; % Read string from cell
    correct = 0;
    for i = 1:N
        user_output = feval(fun);
        solution = feval([fun, '_solution']);
        if almostequal(user_output, solution) 
            correct = correct + 1;
        end
    end
    disp_evaluation(fun, correct, N)
end

% Evaluate functions with q as input
for fun_cell = {'jointToTransform01', ...
                'jointToTransform12', ...
                'jointToTransform23', ...
                'jointToTransform34', ...
                'jointToTransform45', ...
                'jointToTransform56', ...
                'jointToRotMat', ...
                'jointToQuat', ...
                'jointToPosition'}
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
    disp_evaluation(fun, correct, N)
end

% Evaluate quatToRotMat
fun = 'quatToRotMat';
correct = 0;
for i = 1:N
    quat = get_rand_quat();
    user_output = feval(fun, quat);
    solution = feval([fun, '_solution'], quat);
    if almostequal(user_output, solution)
        correct = correct + 1;
    end
end
disp_evaluation(fun, correct, N)

% Evaluate quatMult.m
fun = 'quatMult';
correct = 0;
for i = 1:N
    quat1 = get_rand_quat();
    quat2 = get_rand_quat();
    user_output = feval(fun, quat1, quat2);
    solution = feval([fun, '_solution'], quat1, quat2);
    if almostequal(user_output, solution)
        correct = correct + 1;
    end
end
disp_evaluation(fun, correct, N)

% Evaluate rotMatToQuat.m
fun = 'rotMatToQuat';
correct = 0;
for i = 1:N
    C = get_rand_rot();
    user_output = feval(fun, C);
    solution = feval([fun, '_solution'], C);
    if almostequal(user_output, solution)
        correct = correct + 1;
    end
end
disp_evaluation(fun, correct, N)

% Evaluate rotVecWithQuat.m
fun = 'rotVecWithQuat';
correct = 0;
for i = 1:N
    quat = get_rand_quat();
    vec = randn(3,1);
    user_output = feval(fun, quat, vec);
    solution = feval([fun, '_solution'], quat, vec);
    if almostequal(user_output, solution)
        correct = correct + 1;
    end
end
disp_evaluation(fun, correct, N)

fprintf('... Done \n', N)

