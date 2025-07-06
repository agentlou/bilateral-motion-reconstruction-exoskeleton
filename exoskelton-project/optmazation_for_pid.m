clear; clc;

%% === Load All Trajectories ===
load('learnedHipTrajectory.mat', 'hipTrajectory', 'hipTime');
load('learnedKneeTrajectory.mat', 'kneeTrajectory', 'kneeTime');
load('symmetricHipResults.mat', ...
    'theta_right_hip_ideal', 'theta_right_hip_nonideal');
load('symmetricKneeFromGMR.mat', ...
    'theta_right_knee_ideal', 'theta_right_knee_nonideal');

% Common time base
time = hipTime(:);
timeStep = time(2) - time(1);

% Interpolate all to common time base
kneeTrajectory             = interp1(kneeTime, kneeTrajectory, time, 'linear');
theta_right_knee_ideal     = interp1(kneeTime, theta_right_knee_ideal, time, 'linear');
theta_right_knee_nonideal  = interp1(kneeTime, theta_right_knee_nonideal, time, 'linear');

% Organize desired trajectories
desiredLabels = {
    'Left Knee',            kneeTrajectory;
    'Left Hip',             hipTrajectory;
    'Right Knee (Ideal)',   theta_right_knee_ideal;
    'Right Hip (Ideal)',    theta_right_hip_ideal;
    'Right Knee (Non)',     theta_right_knee_nonideal;
    'Right Hip (Non)',      theta_right_hip_nonideal;
};

nJoints = size(desiredLabels, 1);
actualTrajectories = zeros(length(time), nJoints);
pidGains = zeros(nJoints, 3);

%% === PID Optimization and Simulation ===
initialParams = [5, 1, 0.5];
lb = [0, 0, 0];
ub = [20, 10, 5];

for i = 1:nJoints
    desired = desiredLabels{i,2};
    objFunc = @(params) pidObjective(desired, time, params);
    optimal = fmincon(objFunc, initialParams, [], [], [], [], lb, ub, [], ...
        optimoptions('fmincon','Display','none','Algorithm','sqp'));
    pidGains(i,:) = optimal;

    [actual, ~] = simulateJointPID(time, desired, optimal);
    actualTrajectories(:, i) = actual;
end

%% === Plot Results ===
figure('Name','Full Leg Symmetry + PID','Position',[100 100 1000 700]);
titles = {
    'Left Knee', 'Left Hip';
    'Right Knee (Ideal)', 'Right Hip (Ideal)';
    'Right Knee (Non)', 'Right Hip (Non)'
};
legends = {'Desired','Actual'};
subplotIdx = [1, 3, 5, 2, 4, 6];  % Fix subplot order

for i = 1:nJoints
    subplot(3, 2, subplotIdx(i));
    plot(time, desiredLabels{i,2}, '--r', 'LineWidth', 1.5); hold on;
    plot(time, actualTrajectories(:, i), '-b', 'LineWidth', 1.5);
    title(titles{i}); xlabel('Time (s)'); ylabel('Angle (rad)');
    legend(legends, 'Location', 'best'); grid on;
end
sgtitle('PID-Controlled Joint Trajectories: GMR & Symmetry');

%% === RMS Error Printout ===
fprintf('\n🔍 RMS Errors (PID-Controlled vs Desired):\n');
fprintf('%-25s | %10s\n', 'Joint', 'RMS (rad)');
fprintf(repmat('-',1,40)); fprintf('\n');

for i = 1:nJoints
    desired = desiredLabels{i,2};
    actual  = actualTrajectories(:,i);
    rmsErr  = sqrt(mean((desired - actual).^2));
    fprintf('%-25s | %10.5f\n', desiredLabels{i,1}, rmsErr);
end

%% === Save Results ===
save('pidOptimizedTrajectories.mat', 'actualTrajectories', 'pidGains', 'time');

%% === Function: PID Objective (MSE) ===
function err = pidObjective(desired, time, params)
    desired = desired(:);
    [actual, ~] = simulateJointPID(time, desired, params);
    err = mean((desired - actual).^2);
end

%% === Function: PID Simulation (2nd-order model) ===
function [actual, control] = simulateJointPID(time, desired, pid)
    Kp = pid(1); Ki = pid(2); Kd = pid(3);
    dt = time(2) - time(1);
    n = length(time);
    actual = zeros(n,1);
    velocity = 0;
    control = zeros(n,1);
    integral = 0;
    prevError = 0;
    damping = 0.1;

    for i = 2:n
        err = desired(i) - actual(i-1);
        integral = integral + err * dt;
        derivative = (err - prevError) / dt;

        u = Kp * err + Ki * integral + Kd * derivative;
        velocity = velocity + u * dt - damping * velocity;
        actual(i) = actual(i-1) + velocity * dt;

        control(i) = u;
        prevError = err;
    end
end
