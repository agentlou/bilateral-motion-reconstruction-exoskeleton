function stop = pidOutputFcn(x, optimValues, state)
    stop = false;

    % Store iteration history in global scope
    global optimizationHistory;

    if isequal(state, 'iter')
        % Append [Kp, Ki, Kd, objective value] to history
        optimizationHistory = [optimizationHistory; x, optimValues.fval];

        % Print progress to console
        fprintf('Iteration %d: Kp = %.3f, Ki = %.3f, Kd = %.3f | Obj = %.4f\n', ...
                optimValues.iteration, x(1), x(2), x(3), optimValues.fval);
    end
end
