% Script: generate_nonideal_symmetry_from_left_leg.m
clear; clc;

%% === Load Left Leg Trajectories ===
load('trajectories.mat', 'desiredTrajectories', 'time');
L_hip  = desiredTrajectories(:, 1);
L_knee = desiredTrajectories(:, 2);
t      = time(:);

%% === Trim time to 9s to avoid extrapolation artifacts ===
idx = t <= 9;
t = t(idx);
L_hip = L_hip(idx);
L_knee = L_knee(idx);

%% === Gait Cycle Parameters ===
T_cycle = 1.5;
half_T  = T_cycle / 2;
t_shifted = t + half_T;

%% === Ideal symmetry (for optimization)
R_hip_ideal  = -interp1(t, L_hip,  t_shifted, 'linear', 'extrap');
R_knee_ideal = -interp1(t, L_knee, t_shifted, 'linear', 'extrap');

%% === Non-Ideal Warping Parameters
[alpha_hip, beta_hip, delta_hip, ~]    = optimize_symmetry_params(L_hip,  R_hip_ideal,  t);
[alpha_knee, beta_knee, delta_knee, ~] = optimize_symmetry_params(L_knee, R_knee_ideal, t);

%% === Time-Warp Clipping
warp_time = @(b, d) min(max(b * t + d, t(1)), t(end));
t_warp_hip  = warp_time(beta_hip,  delta_hip);
t_warp_knee = warp_time(beta_knee, delta_knee);

%% === Interpolate and Apply Symmetry
R_hip_nonideal  = -alpha_hip  * interp1(t, L_hip,  t_warp_hip,  'linear');
R_knee_nonideal = -alpha_knee * interp1(t, L_knee, t_warp_knee, 'linear');

% Fill missing data
R_hip_nonideal  = fillmissing(R_hip_nonideal,  'nearest');
R_knee_nonideal = fillmissing(R_knee_nonideal, 'nearest');

%% === RMS Normalization
rms_L_hip = rms(L_hip);  rms_R_hip = rms(R_hip_nonideal);
rms_L_knee = rms(L_knee); rms_R_knee = rms(R_knee_nonideal);

if rms_R_hip > 1e-3
    R_hip_nonideal = R_hip_nonideal * (rms_L_hip / rms_R_hip);
end
if rms_R_knee > 1e-3
    R_knee_nonideal = R_knee_nonideal * (rms_L_knee / rms_R_knee);
end

%% === Save to new filenames (avoids overwriting ideal data)
save('nonidealSymmetricHip.mat', 'R_hip_nonideal');
save('nonidealSymmetricKnee.mat', 'R_knee_nonideal');
disp('✅ Non-ideal symmetry saved to: nonidealSymmetricHip.mat and nonidealSymmetricKnee.mat');

%% === Plot for verification
figure('Name','Non-Ideal Symmetry','Position',[100 100 1000 500]);

subplot(1,2,1);
plot(t, L_hip, 'k--', 'LineWidth', 1.5); hold on;
plot(t, R_hip_ideal, 'b-', 'LineWidth', 1.5);
plot(t, R_hip_nonideal, 'r-', 'LineWidth', 1.5);
title('Hip Symmetry'); legend('Left', 'Right Ideal', 'Right Non-Ideal');
ylabel('Angle (rad)'); xlabel('Time (s)'); grid on;

subplot(1,2,2);
plot(t, L_knee, 'k--', 'LineWidth', 1.5); hold on;
plot(t, R_knee_ideal, 'b-', 'LineWidth', 1.5);
plot(t, R_knee_nonideal, 'r-', 'LineWidth', 1.5);
title('Knee Symmetry'); legend('Left', 'Right Ideal', 'Right Non-Ideal');
ylabel('Angle (rad)'); xlabel('Time (s)'); grid on;

sgtitle('✅ Non-Ideal Right Leg via Time-Warped Symmetry (Clipped & Scaled)');
