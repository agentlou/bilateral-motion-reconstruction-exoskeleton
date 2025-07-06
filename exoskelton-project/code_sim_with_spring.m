clear; clc;

%% === Load Trajectories ===
load('learnedHipTrajectory.mat', 'hipTrajectory', 'hipTime');
load('learnedKneeTrajectory.mat', 'kneeTrajectory', 'kneeTime');
load('symmetricHipResults.mat', 'theta_right_hip_ideal');
load('symmetricKneeFromGMR.mat', 'theta_right_knee_ideal');
load('optimized_pid_full_leg.mat', 'optimalParams', 'time', 'timeStep');
load('robot_setup.mat', 'jointUpperLimits', 'jointLowerLimits');

%% === Interpolate to match common time ===
nSteps = length(time);
commonTime = linspace(0, 1, nSteps);

hipTrajectory = interp1(linspace(0,1,length(hipTrajectory)), hipTrajectory, commonTime)';
kneeTrajectory = interp1(linspace(0,1,length(kneeTrajectory)), kneeTrajectory, commonTime)';
theta_right_hip_ideal = interp1(linspace(0,1,length(theta_right_hip_ideal)), theta_right_hip_ideal, commonTime)';
theta_right_knee_ideal = interp1(linspace(0,1,length(theta_right_knee_ideal)), theta_right_knee_ideal, commonTime)';

% Convert to radians if needed
if max(abs(hipTrajectory)) > 10
    hipTrajectory = deg2rad(hipTrajectory);
    kneeTrajectory = deg2rad(kneeTrajectory);
    theta_right_hip_ideal = deg2rad(theta_right_hip_ideal);
    theta_right_knee_ideal = deg2rad(theta_right_knee_ideal);
    disp('✅ Converted all trajectories to radians.');
end

%% === Desired Trajectories Matrix ===
% Format: [L_hip, L_knee, R_hip, R_knee]
desiredTrajectories = [hipTrajectory, kneeTrajectory, ...
                       theta_right_hip_ideal, theta_right_knee_ideal];

%% === Simulation Parameters ===
springStiffness = 17;  % Spring constant (Nm/rad)
jointInertias = [0.02, 0.01, 0.02, 0.01];  % Example values
nJoints = 4;

% Initial state
config = zeros(1, nJoints);
velocity = zeros(1, nJoints);
integralError = zeros(1, nJoints);
previousError = zeros(1, nJoints);
actualTrajectory = zeros(nSteps, nJoints);

% PID gains
Kp = reshape(optimalParams(1:4, 1), 1, []);
Ki = reshape(optimalParams(1:4, 2), 1, []);
Kd = reshape(optimalParams(1:4, 3), 1, []);

%% === Simulation Loop ===
for t = 1:nSteps
    desired = desiredTrajectories(t, :);
    error = desired - config;
    derivative = (error - previousError) / timeStep;

    % PID control
    torquePID = Kp .* error + Ki .* integralError + Kd .* derivative;

    % Spring effect (simple joint restoring)
    torqueSpring = -springStiffness * config;

    % Total torque
    totalTorque = torquePID + torqueSpring;

    for j = 1:nJoints
        acc = totalTorque(j) / jointInertias(j);
        velocity(j) = velocity(j) + acc * timeStep;
        config(j) = config(j) + velocity(j) * timeStep;

        % Apply joint limits
        config(j) = max(min(config(j), jointUpperLimits(j)), jointLowerLimits(j));
    end

    actualTrajectory(t, :) = config;
    previousError = error;
    integralError = integralError + error * timeStep;
end

%% === Save Results ===
save('actualTrajectoryWithSpring_PID.mat', 'actualTrajectory', 'time');
disp('✅ Simulation with PID + Spring completed and saved.');

%% === RMS Error Report ===
jointNames = {'L Hip', 'L Knee', 'R Hip (Ideal)', 'R Knee (Ideal)'};
fprintf('\n🔍 RMS Errors (Simulated vs Desired):\n');
for i = 1:nJoints
    des = desiredTrajectories(:, i);
    act = actualTrajectory(:, i);
    rmsErr = sqrt(mean((des - act).^2));
    fprintf('%-20s: %.5f rad\n', jointNames{i}, rmsErr);
end
