clear; clc;

% === Load GMR for left leg ===
load('learnedHipTrajectory.mat', 'hipTrajectory', 'hipTime');
gmr_hip = hipTrajectory;
t_hip = hipTime;

load('learnedKneeTrajectory.mat', 'kneeTrajectory', 'kneeTime');
gmr_knee = kneeTrajectory;
t_knee = kneeTime;

% === Load symmetry results ===
load('symmetricHipResults.mat', ...
    'theta_right_hip_ideal', 'theta_right_hip_nonideal');

load('symmetricKneeFromGMR.mat', ...
    'theta_right_knee_ideal', 'theta_right_knee_nonideal');

% === Create figure with correct subplot layout ===
figure('Name','Leg Joint Trajectories (GMR + Symmetry)','Position',[100 100 1000 600]);

% === Row 1: Left side ===
subplot(3,2,1); % Left Knee (GMR)
plot(t_knee, gmr_knee, 'k', 'LineWidth', 2);
title('Left Knee (GMR)');
ylabel('Angle (rad)');
grid on;

subplot(3,2,2); % Left Hip (GMR)
plot(t_hip, gmr_hip, 'k', 'LineWidth', 2);
title('Left Hip (GMR)');
grid on;

% === Row 2: Ideal symmetry ===
subplot(3,2,3); % Right Knee (Ideal)
plot(t_knee, theta_right_knee_ideal, 'b', 'LineWidth', 2);
title('Right Knee (Ideal Symmetry)');
ylabel('Angle (rad)');
grid on;

subplot(3,2,4); % Right Hip (Ideal)
plot(t_hip, theta_right_hip_ideal, 'b', 'LineWidth', 2);
title('Right Hip (Ideal Symmetry)');
grid on;

% === Row 3: Non-Ideal symmetry ===
subplot(3,2,5); % Right Knee (Non-Ideal)
plot(t_knee, theta_right_knee_nonideal, 'r', 'LineWidth', 2);
title('Right Knee (Non-Ideal Symmetry)');
ylabel('Angle (rad)');
xlabel('Time (s)');
grid on;

subplot(3,2,6); % Right Hip (Non-Ideal)
plot(t_hip, theta_right_hip_nonideal, 'r', 'LineWidth', 2);
title('Right Hip (Non-Ideal Symmetry)');
xlabel('Time (s)');
grid on;

% Global title
sgtitle('GMR & Symmetry-Based Trajectories: Knee vs. Hip');

disp('✅ Plot generated with correct knee/hip layout.');
