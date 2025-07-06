% Script: generate_symmetric_from_left_leg.m
clear; clc;

%% === Load Left Leg Trajectories ===
load('trajectories.mat', 'desiredTrajectories', 'time', 'timeStep');
L_hip  = desiredTrajectories(:, 1);
L_knee = desiredTrajectories(:, 2);
t      = time(:);

%% === Trim at t = 9s ===
idx = t <= 9;
t = t(idx);
L_hip = L_hip(idx);
L_knee = L_knee(idx);

%% === Define Gait Cycle Period and Phase Shift ===
T_cycle = 1.5;                % Gait cycle duration
t_shifted = t + T_cycle / 2;  % Phase shift for symmetry

%% === Generate Ideal Symmetric Right Leg ===
R_hip_ideal  = -interp1(t, L_hip,  t_shifted, 'linear', 'extrap');
R_knee_ideal = -interp1(t, L_knee, t_shifted, 'linear', 'extrap');

%% === Save Symmetric Results ===
save('symmetricHipResults.mat', 'R_hip_ideal');
save('symmetricKneeFromGMR.mat', 'R_knee_ideal');

disp('✅ Ideal symmetric trajectories (up to 9s) generated and saved.');

%% === Plot to Verify ===
figure('Name', 'Symmetric Gait Trajectories from Left Leg', 'Position', [100 100 900 600]);

subplot(2,2,1);
plot(t, L_hip, 'b', 'LineWidth', 2);
title('Left Hip'); ylabel('Angle (rad)'); xlabel('Time (s)'); grid on;

subplot(2,2,2);
plot(t, L_knee, 'r', 'LineWidth', 2);
title('Left Knee'); ylabel('Angle (rad)'); xlabel('Time (s)'); grid on;

subplot(2,2,3);
plot(t, R_hip_ideal, 'b', 'LineWidth', 2);
title('Right Hip (Ideal)'); ylabel('Angle (rad)'); xlabel('Time (s)'); grid on;

subplot(2,2,4);
plot(t, R_knee_ideal, 'r', 'LineWidth', 2);
title('Right Knee (Ideal)'); ylabel('Angle (rad)'); xlabel('Time (s)'); grid on;
