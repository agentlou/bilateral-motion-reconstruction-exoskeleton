% === STABILIZED ENERGY TANK CONTROL SYSTEM ===
clear; clc;

%% === LOAD DATA ===
load('robot_setup.mat');
load('trajectories.mat', 'desiredTrajectories', 'time', 'timeStep');
load('symmetricHipResults.mat', 'R_hip_ideal');
load('symmetricKneeFromGMR.mat', 'R_knee_ideal');

%% === TRIM TO COMMON LENGTH ===
minLength = min([length(time), length(R_hip_ideal), length(R_knee_ideal), size(desiredTrajectories,1)]);
time = time(1:minLength);
desiredTrajectories = desiredTrajectories(1:minLength, :);
R_hip_ideal = R_hip_ideal(1:minLength);
R_knee_ideal = R_knee_ideal(1:minLength);
desiredTrajectories = [desiredTrajectories(:, 1:2), R_hip_ideal, R_knee_ideal];

%% === SETUP ===
nJoints = 4;
nSteps = length(time);
actualTrajectories = zeros(nSteps, nJoints);
jointVelocities = zeros(1, nJoints);
jointInertias = [0.01, 0.02, 0.01, 0.02];
gamma_history = zeros(nSteps, nJoints);

%% === ENERGY TANK CONTROL PARAMS ===
alpha = [0.2, 0.4, 0.2, 0.4];  % tuned injection gain
E_max = [5, 8, 5, 8];
E_min = [0.5, 0.5, 0.5, 0.5];
E_tank = ones(1, nJoints) .* 5;
E_tank_history = zeros(nSteps, nJoints);
P_in_history = zeros(nSteps, nJoints);
P_out_history = zeros(nSteps, nJoints);
trackingError = zeros(nSteps, nJoints);
torque_history = zeros(nSteps, nJoints);

% control gains
Kp = 4;  % virtual stiffness
Kd = 1.5;  % virtual damping
tau_limit = 30;  % torque saturation

%% === SIMULATION LOOP ===
for tIdx = 1:nSteps
    for j = 1:nJoints
        q = actualTrajectories(tIdx, j);
        q_d = desiredTrajectories(tIdx, j);
        dq = jointVelocities(j);

        if tIdx > 1
            dq_d = (desiredTrajectories(tIdx,j) - desiredTrajectories(tIdx-1,j)) / timeStep;
        else
            dq_d = 0;
        end

        % --- Virtual control torque ---
        tau_d = Kp * (q_d - q) - Kd * dq;
        tau_d = max(min(tau_d, tau_limit), -tau_limit);  % limit

        % --- Energy dynamics ---
        P_in = alpha(j) * abs(tau_d * dq_d);
        P_out = tau_d * dq; if P_out < 0, P_out = 0; end
        dE = (P_in - P_out);
        E_tank(j) = min(max(E_tank(j) + dE * timeStep, E_min(j)), E_max(j));

        % --- Apply energy-based scaling ---
        gamma = min(1, E_tank(j) / E_max(j));
        tau_final = gamma * tau_d;

        % --- Integrate motion ---
        acc = tau_final / jointInertias(j);
        jointVelocities(j) = jointVelocities(j) + acc * timeStep;
        if tIdx < nSteps
            actualTrajectories(tIdx+1, j) = actualTrajectories(tIdx, j) + jointVelocities(j) * timeStep;
        end

        %% Logging
        E_tank_history(tIdx, j) = E_tank(j);
        P_in_history(tIdx, j) = P_in;
        P_out_history(tIdx, j) = P_out;
        trackingError(tIdx, j) = abs(q_d - q);
        gamma_history(tIdx, j) = gamma;
        torque_history(tIdx, j) = tau_final;
    end
end

%% === PLOTS ===
jointLabels = {'Left Hip', 'Left Knee', 'Right Hip', 'Right Knee'};

% Tracking
figure('Name','Tracking Performance');
tiledlayout(2,2);
for j = 1:nJoints
    nexttile; hold on;
    plot(time, desiredTrajectories(:,j), '--', 'LineWidth', 1.5);
    plot(time, actualTrajectories(:,j), '-', 'LineWidth', 1.5);
    title(jointLabels{j}); xlabel('Time (s)'); ylabel('Angle (rad)'); legend('Desired','Actual'); grid on;
end

% Energy Metrics
figure('Name','Energy Tank Metrics');
tiledlayout(3,1);
nexttile; plot(time, E_tank_history, 'LineWidth', 1.5); title('Energy Tank Levels'); ylabel('Energy'); legend(jointLabels); grid on;
nexttile; plot(time, P_in_history, 'LineWidth', 1.5); title('Injected Power (P_{in})'); ylabel('W'); legend(jointLabels); grid on;
nexttile; plot(time, P_out_history, 'LineWidth', 1.5); title('Output Power (P_{out})'); xlabel('Time (s)'); ylabel('W'); legend(jointLabels); grid on;

% Gamma
figure('Name','Gamma Scaling');
plot(time, gamma_history, 'LineWidth', 1.5); title('Effort Scaling γ'); xlabel('Time (s)'); ylabel('γ'); legend(jointLabels); grid on;

% Torque
figure('Name','Final Torque');
plot(time, torque_history, 'LineWidth', 1.5); title('Applied Torque per Joint'); xlabel('Time (s)'); ylabel('Torque (Nm)'); legend(jointLabels); grid on;

% Tracking Error
figure('Name','Tracking Error');
plot(time, trackingError, 'LineWidth', 1.5); title('Tracking Error |q_d - q|'); xlabel('Time (s)'); ylabel('rad'); legend(jointLabels); grid on;

%% === RMS Tracking Summary ===
rmsErrors = sqrt(mean((desiredTrajectories - actualTrajectories).^2));
disp('=== RMS Tracking Errors (rad) ===');
disp(table(jointLabels', rmsErrors', 'VariableNames', {'Joint', 'RMS_Error'}));
