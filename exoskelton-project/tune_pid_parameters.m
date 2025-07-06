function bestParams = tune_pid_parameters(desiredTraj, actualTrajFunc, timeStep)
    % Optimize Kp, Ki, Kd per joint using fminsearch
    nJoints = size(desiredTraj, 2);

    bestParams = zeros(nJoints, 3);  % [Kp, Ki, Kd] for each joint
    options = optimset('Display', 'iter', 'MaxIter', 100);

    for j = 1:nJoints
        costFunc = @(gains) simulateAndComputeRMSError(gains, desiredTraj(:, j), actualTrajFunc, timeStep, j);
        bestParams(j, :) = fminsearch(costFunc, [10, 1, 1], options);
    end
end

function err = simulateAndComputeRMSError(gains, desired, simFunc, dt, jointIdx)
    % Simulate a single joint with given PID gains and return RMS error
    [actual, ~] = simFunc(gains, dt, jointIdx, desired);
    err = sqrt(mean((desired - actual).^2));
end
