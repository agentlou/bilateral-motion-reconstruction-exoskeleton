% Script 1: Initialization and Robot Setup

% Load the URDF
urdfFile = 'C:\Users\bench\OneDrive\Desktop\new_test\urdf\exoskeleton.urdf';
robot = importrobot(urdfFile);

% Set properties for simulation
robot.DataFormat = 'row';
robot.Gravity = [0, 0, -9.81];

% Visualize the robot in its initial pose
figure;
show(robot, 'Visuals', 'on');
title('Exoskeleton Robot Initial Pose');
grid on;

% Define joint limits based on URDF file
jointLowerLimits = [-0.87, 0, -0.87, 0]; % Lower bounds (radians)
jointUpperLimits = [1.57, 2.09, 1.57, 2.09]; % Upper bounds (radians)

% Save essential data to a .mat file for use in other scripts
save('robot_setup.mat', 'robot', 'jointLowerLimits', 'jointUpperLimits');

disp('Initialization and setup completed.');
