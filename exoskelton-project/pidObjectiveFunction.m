function rmseError = pidObjectiveFunction(params, robot, time, desiredTrajectories, jointLowerLimits, jointUpperLimits)
    % --- PID gains for Joint 2 (Left Knee) ---
    Kp = params(1);
    Ki = params(2);
    Kd = params(3);

    % --- Initial States ---
    config = 0;                 % Initial joint angle
    velocity = 0;              % Optional: add dynamics realism
    integralError = 0;
    previousError = 0;

    % --- Time Setup ---
    dt = time(2) - time(1);    % Time step
    nSteps = length(time);
    actualTrajectory = zeros(nSteps, 1);

    % --- Simulate PID over time ---
    for tIdx = 1:nSteps
        desiredPos = desiredTrajectories(tIdx, 2);  % Joint 2

        errorVal = desiredPos - config;
        integralError = integralError + errorVal * dt;
        derivative = (errorVal - previousError) / dt;

        % PID output (torque or position delta)
        controlSignal = Kp * errorVal + Ki * integralError + Kd * derivative;

        % Add a simple dynamics model: integrate velocity
        velocity = velocity + controlSignal * dt;
        config = config + velocity * dt;

        % Enforce joint limits
        config = min(max(config, jointLowerLimits(2)), jointUpperLimits(2));

        % Save result
        actualTrajectory(tIdx) = config;

        % Update error
        previousError = errorVal;
    end

    % --- Compute RMSE ---
    rmseError = sqrt(mean((desiredTrajectories(:, 2) - actualTrajectory).^2));
end
