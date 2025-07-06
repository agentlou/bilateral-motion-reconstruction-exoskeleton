% === Load Left Leg Trajectory Data ===
load('trajectories.mat');  % desiredTrajectories and time

% Extract variables
t = time;
theta_left_hip  = desiredTrajectories(:, 1);
theta_left_knee = desiredTrajectories(:, 2);

% Define gait cycle duration and half-cycle shift
T_cycle = 2.0;
half_shift = T_cycle / 2;

% Apply ideal symmetry: θ_R(t) = -θ_L(t + T/2)
theta_right_hip  = -interp1(t, theta_left_hip,  t + half_shift,  'spline', 'extrap');
theta_right_knee = -interp1(t, theta_left_knee, t + half_shift, 'spline', 'extrap');

% Assemble full trajectory
idealSymTrajectories = zeros(length(t), 4);
idealSymTrajectories(:, 1) = theta_left_hip;
idealSymTrajectories(:, 2) = theta_left_knee;
idealSymTrajectories(:, 3) = theta_right_hip;
idealSymTrajectories(:, 4) = theta_right_knee;

% Save
save('ideal_symmetric_trajectories.mat', 'idealSymTrajectories', 'time');
disp('✅ Ideal symmetry trajectory generated and saved.');
