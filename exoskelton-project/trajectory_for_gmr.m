%% === Load Data ===
load('trajectories.mat');                  % desiredTrajectories, time, actualTrajectoryOptimized
load('trajectories_with_springs.mat');     % actualTrajectoryWithSprings
load('fuzzy_pid_output.mat');              % actualTrajectories → fuzzy

% Joint indices
hipIdx = 1;
kneeIdx = 2;

% Extract joint trajectories
desired_hip   = desiredTrajectories(:, hipIdx);
desired_knee  = desiredTrajectories(:, kneeIdx);

actual_fuzzy_hip   = actualTrajectories(:, hipIdx);
actual_fuzzy_knee  = actualTrajectories(:, kneeIdx);

actual_pid_hip     = actualTrajectoryWithSprings(:, hipIdx);
actual_pid_knee    = actualTrajectoryWithSprings(:, kneeIdx);

actual_opt_hip     = actualTrajectoryOptimized(:, hipIdx);
actual_opt_knee    = actualTrajectoryOptimized(:, kneeIdx);

% RMS Errors
rms_hip = [ ...
    sqrt(mean((desired_hip - actual_pid_hip).^2)), ...
    sqrt(mean((desired_hip - actual_fuzzy_hip).^2)), ...
    sqrt(mean((desired_hip - actual_opt_hip).^2)) ];

rms_knee = [ ...
    sqrt(mean((desired_knee - actual_pid_knee).^2)), ...
    sqrt(mean((desired_knee - actual_fuzzy_knee).^2)), ...
    sqrt(mean((desired_knee - actual_opt_knee).^2)) ];

%% === Plot ===
figure('Units','normalized','OuterPosition',[0 0 0.6 1]);

tiledlayout(2, 1, 'TileSpacing','compact', 'Padding','compact');

jointNames = {'Left Hip (Joint 1)', 'Left Knee (Joint 2)'};
yLimits = [-0.7 0.4; 0 2.2];  % Y-axis limits: [min max] for hip & knee

for i = 1:2
    nexttile;
    hold on; box on;

    % Select joint
    if i == 1
        plot(time, desired_hip, '--k', 'LineWidth', 1.5, 'DisplayName', 'Desired');
        plot(time, actual_pid_hip, '-', 'Color', [0 0.45 0.74], 'LineWidth', 1.5, ...
            'DisplayName', sprintf('PID limitation (%.3f)', rms_hip(1)));
        plot(time, actual_fuzzy_hip, '-', 'Color', [0.85 0.33 0.1], 'LineWidth', 1.5, ...
            'DisplayName', sprintf('Fuzzy PID (%.3f)', rms_hip(2)));
        plot(time, actual_opt_hip, '-', 'Color', [0.49 0.18 0.56], 'LineWidth', 1.5, ...
            'DisplayName', sprintf('PID (%.3f)', rms_hip(3)));
    else
        plot(time, desired_knee, '--k', 'LineWidth', 1.5, 'DisplayName', 'Desired');
        plot(time, actual_pid_knee, '-', 'Color', [0 0.45 0.74], 'LineWidth', 1.5, ...
            'DisplayName', sprintf('PID limitation (%.3f)', rms_knee(1)));
        plot(time, actual_fuzzy_knee, '-', 'Color', [0.85 0.33 0.1], 'LineWidth', 1.5, ...
            'DisplayName', sprintf('Fuzzy PID (%.3f)', rms_knee(2)));
        plot(time, actual_opt_knee, '-', 'Color', [0.49 0.18 0.56], 'LineWidth', 1.5, ...
            'DisplayName', sprintf('PID (%.3f)', rms_knee(3)));
    end

    title(jointNames{i}, 'FontName', 'Times New Roman', 'FontSize', 16, 'FontWeight', 'bold');
    xlabel('Time (s)', 'FontName', 'Times New Roman', 'FontSize', 14);
    ylabel('Angle (rad)', 'FontName', 'Times New Roman', 'FontSize', 14);
    ylim(yLimits(i, :));

    legend('Location', 'northeast', 'FontSize', 10);

    ax = gca;
    ax.FontName = 'Times New Roman';
    ax.FontSize = 13;
    ax.FontWeight = 'bold';
    ax.LineWidth = 1.3;
    ax.TickDir = 'out';
    grid on;
end

disp('✅ Final clean comparison plot (Hip + Knee) generated.');
