% === Prepare Data ===
% Assumes:
% t → time vector
% gmr_knee, gmr_hip → left leg GMR-generated trajectories
% theta_right_knee_ideal, theta_right_knee_nonideal
% theta_right_hip_ideal, theta_right_hip_nonideal

% Reflect right leg symmetries to left side
theta_left_knee_ideal_reflected = -theta_right_knee_ideal;
theta_left_knee_nonideal_reflected = -theta_right_knee_nonideal;

theta_left_hip_ideal_reflected = -theta_right_hip_ideal;
theta_left_hip_nonideal_reflected = -theta_right_hip_nonideal;

% Colors (optional)
colors.gmr      = [0 0.45 0.74];
colors.ideal    = [0.49 0.18 0.56];
colors.nonideal = [0.85 0.33 0.1];

% === Plot ===
figure('Units','normalized','OuterPosition',[0 0 0.5 1]);
tiledlayout(2,1,'TileSpacing','compact','Padding','compact');

% --- Left Knee ---
nexttile; hold on;
plot(t, gmr_knee, '--', 'Color', colors.gmr, 'LineWidth', 1.5, 'DisplayName', 'GMR Left Knee');
plot(t, theta_left_knee_ideal_reflected, '--', 'Color', colors.ideal, 'LineWidth', 1.5, 'DisplayName', 'Ideal Symmetry');
plot(t, theta_left_knee_nonideal_reflected, '-', 'Color', colors.nonideal, 'LineWidth', 1.8, 'DisplayName', 'Non-Ideal Symmetry');
title('Left Knee'); ylabel('Angle (rad)');
legend('Location','northeast','FontSize',10); grid on;
xlim([0 8]);  % Limit to 9 seconds

% --- Left Hip ---
nexttile; hold on;
plot(t, gmr_hip, '--', 'Color', colors.gmr, 'LineWidth', 1.5, 'DisplayName', 'GMR Left Hip');
plot(t, theta_left_hip_ideal_reflected, '--', 'Color', colors.ideal, 'LineWidth', 1.5, 'DisplayName', 'Ideal Symmetry');
plot(t, theta_left_hip_nonideal_reflected, '-', 'Color', colors.nonideal, 'LineWidth', 1.8, 'DisplayName', 'Non-Ideal Symmetry');
title('Left Hip'); xlabel('Time (s)'); ylabel('Angle (rad)');
ylim([-0.6 0.4]);  % Keep Y-scale small for hip
legend('Location','northeast','FontSize',10); grid on;
xlim([0 8]);  % Limit to 9 seconds
