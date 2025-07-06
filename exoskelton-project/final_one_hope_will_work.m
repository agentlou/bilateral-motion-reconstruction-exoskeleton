% Script: Fuzzy PID Simulation for Full Leg (4 joints using symmetry)

%% === LOAD DATA ===
clear; clc;
load('robot_setup.mat');                       % Contains 'robot'
load('trajectories.mat', 'desiredTrajectories', 'time', 'timeStep');  % Left Hip & Knee
load('symmetricHipResults.mat', 'R_hip_ideal');
load('symmetricKneeFromGMR.mat', 'R_knee_ideal');
fuzzyController = readfis('fuzzy.fis');        % Load FIS

%% === Trim all to common length ===
minLength = min([length(time), length(R_hip_ideal), length(R_knee_ideal), size(desiredTrajectories,1)]);
time = time(1:minLength);
desiredTrajectories = desiredTrajectories(1:minLength, :);
R_hip_ideal = R_hip_ideal(1:minLength);
R_knee_ideal = R_knee_ideal(1:minLength);

% Combine all 4 joints: [L_Hip, L_Knee, R_Hip, R_Knee]
desiredTrajectories = [desiredTrajectories(:, 1:2), R_hip_ideal, R_knee_ideal];

%% === SETUP ===
nJoints = 4;
nSteps = length(time);
actualTrajectories = zeros(nSteps, nJoints);
jointVelocities = zeros(1, nJoints);
integralError = zeros(1, nJoints);
previousError = zeros(1, nJoints);
jointInertias = [0.01, 0.02, 0.01, 0.02];  % [L_Hip, L_Knee, R_Hip, R_Knee]

% Base PID gains
baseKp = 10; baseKi = 5; baseKd = 2;

%% === Fuzzy PID Simulation ===
for tIdx = 1:nSteps
    for j = 1:nJoints
        error = desiredTrajectories(tIdx, j) - actualTrajectories(tIdx, j);
        deltaError = (tIdx > 1) * (error - previousError(j));

        errorClamped = max(min(error, 3), -3);
        deltaErrorClamped = max(min(deltaError, 3), -3);

        fuzzyInput = [errorClamped, deltaErrorClamped];
        fuzzyOutput = evalfis(fuzzyController, fuzzyInput);

        Kp = baseKp + fuzzyOutput(1);
        Ki = baseKi + fuzzyOutput(2);
        Kd = baseKd + fuzzyOutput(3);

        integralError(j) = integralError(j) + error * timeStep;
        derivative = deltaError / timeStep;
        controlSignal = Kp * error + Ki * integralError(j) + Kd * derivative;

        acceleration = controlSignal / jointInertias(j);
        jointVelocities(j) = jointVelocities(j) + acceleration * timeStep;

        if tIdx < nSteps
            actualTrajectories(tIdx + 1, j) = actualTrajectories(tIdx, j) + jointVelocities(j) * timeStep;
        end

        previousError(j) = error;
    end
end

%% === SAVE OUTPUT ===
save('fuzzy_pid_output_full_leg.mat', 'actualTrajectories', 'time', 'timeStep');
disp('✅ Fuzzy PID simulation completed and saved for 4 joints.');

%% === PLOT RESULTS ===
jointLabels = {'Left Hip', 'Left Knee', 'Right Hip', 'Right Knee'};
colors = {[0 0.45 0.74], [0.85 0.33 0.1]};
rmsErrors = zeros(1, nJoints);  % Store RMS values

figure('Units','inches','Position',[1, 1, 9, 7], 'PaperPositionMode', 'auto');
tiledlayout(2, 2, 'Padding', 'compact', 'TileSpacing', 'compact');

for j = 1:nJoints
    rms_error = sqrt(mean((desiredTrajectories(:, j) - actualTrajectories(:, j)).^2));
    rmsErrors(j) = rms_error;

    nexttile;
    hold on; box on;
    plot(time, desiredTrajectories(:, j), '--', 'Color', colors{1}, 'LineWidth', 1.5, 'DisplayName', 'Desired');
    plot(time, actualTrajectories(:, j), '-',  'Color', colors{2}, 'LineWidth', 1.5, 'DisplayName', 'Fuzzy PID');
    title(sprintf('%s\nRMS Error = %.4f rad', jointLabels{j}, rms_error), ...
          'FontName', 'Times New Roman', 'FontSize', 14, 'FontWeight', 'bold');
    xlabel('Time (s)', 'FontSize', 12);
    ylabel('Angle (rad)', 'FontSize', 12);
    legend('Location', 'northeast', 'FontSize', 10);
    grid on;

    ax = gca;
    ax.FontName = 'Times New Roman';
    ax.FontSize = 12;
    ax.FontWeight = 'bold';
    ax.LineWidth = 1.2;
    ax.TickDir = 'out';
end

print(gcf, 'fig_fuzzy_pid_full_leg.pdf', '-dpdf', '-bestfit');
disp('✅ Full-leg fuzzy PID plot saved to PDF.');

%% === Display RMS Errors as Table ===
fprintf('\n=== RMS Error Table (rad) ===\n');
rmsTable = table(jointLabels', rmsErrors', ...
    'VariableNames', {'Joint', 'RMS_Error'});
disp(rmsTable);
