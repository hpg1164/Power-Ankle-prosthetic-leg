% Load Data
robot_data_1 = load('data_4.txt'); 
robot_data_2 = load('data_2.txt'); 
robot_data_3 = load('IMU_data_3.csv'); 
robot_data_4 = load('IMU_data_4.csv'); 
robot_data_5 = load('IMU_data_5.csv'); 

% Define Time Vectors
time_robot_1 = linspace(0, 1.5, length(robot_data_1)); 
time_robot_2 = linspace(0, 1.5, length(robot_data_2)); 
time_robot_3 = linspace(0, 1.5, length(robot_data_3)); 
time_robot_4 = linspace(0, 1.5, length(robot_data_4)); 
time_robot_5 = linspace(0, 1.5, length(robot_data_5)); 

% Plot Data
figure; hold on;
plot(time_robot_1, robot_data_1, 'r-', 'LineWidth', 2); 
plot(time_robot_2, robot_data_2, 'b-', 'LineWidth', 2); 
plot(time_robot_3, robot_data_3, 'g-', 'LineWidth', 2); 
plot(time_robot_4, robot_data_4, 'm-', 'LineWidth', 2);  % 'm-' for magenta
plot(time_robot_5, robot_data_5, 'k-', 'LineWidth', 2);  % 'k-' for black

% Add labels, title, and legend
title('Robot Data');
xlabel('Time [s]');
ylabel('Angle (Degree)');
legend('Robot Data 1', 'Robot Data 2', 'Robot Data 3', 'Robot Data 4', 'Robot Data 5');
grid on;
hold off;
