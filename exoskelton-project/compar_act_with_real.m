clear; clc;

%% === Load Data ===
load('actualTrajectoryWithSpring_PID.mat', 'actualTrajectory', 'time');   % Simulated (Spring + PID)
load('learnedHipTrajectory.mat', 'hipTrajectory', 'hipTime');             % Left Hip desired
load('learnedKneeTrajectory.mat', 'kneeTrajectory', 'kneeTime');
load('symmetricHipResults.mat', 'theta_right_hip_ideal');
load('symmetricKneeFromGMR.mat', 'theta_right_knee_ideal');

%% === Interpolate Desired Trajectories to Match Time ===
hipTrajectory           = interp1(hipTime, hipTrajectory, time, 'linear')';
kneeTrajectory          = interp1(kneeTime, kneeTrajectory, time, 'linear')';
theta_right_hip_ideal   = interp1(hipTime, theta_right_hip_ideal, time, 'linear')';
theta_right_knee_ideal  = interp1(kneeTime, theta_right_knee_ideal, time, 'linear')';

%% === Convert to Radians If Needed ===
if max(abs(hipTrajectory)) > 10, hipTrajectory = deg2rad(hipTrajectory); end
if max(abs(kneeTrajectory)) > 10, kneeTrajectory = deg2rad(kneeTrajectory); end
if max(abs(theta_right_hip_ideal)) > 10, theta_right_hip_ideal = deg2rad(theta_right_hip_ideal); end
if max(abs(theta_right_knee_ideal)) > 10, theta_right_knee_ideal = deg2rad(theta_right_knee_ideal); end

%% === Desired Trajectories Matrix ===
desiredTrajectories = [hipTrajectory, kneeTrajectory, ...
                       theta_right_hip_ideal, theta_right_knee_ideal];

jointLabels = {'Left Hip', 'Left Knee', 'Right Hip', 'Right Knee'};

%% === Plot Settings ===
colors = {[0 0 1], [1 0 0]};  % Blue (Desired), Red (Simulated)

figure('Units','inches','Position',[1 1 8 6], 'PaperPositionMode','auto');
tiledlayout(2, 2, 'Padding', 'tight', 'TileSpacing', 'compact');

for i = 1:4
    desired = desiredTrajectories(:, i);
    simulated = actualTrajectory(:, i);

    % Check size match
    if length(desired) ~= length(simulated)
        error('❌ Size mismatch at joint %d: desired = %d, simulated = %d', ...
              i, length(desired), length(simulated));
    end

    % RMS error
    rms_error = sqrt(mean((desired - simulated).^2));

    % Plot
    nexttile;
    hold on; box on;
    plot(time, desired, '-', 'Color', colors{1}, 'LineWidth', 1.8, 'DisplayName', 'Desired');
    plot(time, simulated, '-', 'Color', colors{2}, 'LineWidth', 1.8, ...
         'DisplayName', sprintf('Simulated (RMS = %.3f)', rms_error));

    title(jointLabels{i}, 'FontName', 'Times New Roman', 'FontSize', 14, 'FontWeight', 'bold');
    xlabel('Time (s)', 'FontName', 'Times New Roman', 'FontSize', 12);
    ylabel('Angle (rad)', 'FontName', 'Times New Roman', 'FontSize', 12);
    legend('Location', 'northeast', 'FontSize', 10);

    % Axes styling
    ax = gca;
    ax.FontName = 'Times New Roman';
    ax.FontSize = 12;
    ax.FontWeight = 'bold';
    ax.LineWidth = 1.3;
    ax.TickDir = 'out';
    grid on;
end

%% === Export as High-Quality PDF ===
outputFile = 'fig_4joints_des_vs_sim_2x2.pdf';
print(gcf, outputFile, '-dpdf', '-r600');
disp(['✅ Full 2x2 plot saved: ', outputFile]);
