% Load the data
gait_data = load('data.txt'); 
robot_data = load('data_4.txt'); 

% Define the filter
fs = length(robot_data) / 1.5;  % Sampling frequency (estimated from data length)
fc = 5; % Cutoff frequency in Hz (adjust as needed)
[b, a] = butter(4, fc/(fs/2), 'low'); % 4th-order low-pass Butterworth filter

% Apply the filter to the robot data
filtered_robot_data = filtfilt(b, a, robot_data);

% Time vectors
time_gait = linspace(0, 1.5, length(gait_data));
time_robot = linspace(0, 1.5, length(robot_data));

% Plot results
figure;
plot(time_gait, gait_data, 'b-', 'LineWidth', 2);
hold on;
plot(time_robot, filtered_robot_data, 'r-', 'LineWidth', 2);
title('Robotic Leg: Gait Data vs. Robot Data (Filtered)','FontSize',20);
xlabel('Time[s]','FontSize',	20);
ylabel('Angle (Degree)','FontSize',20);
legend('Gait Data', 'Filtered Robot Data');
grid on;
hold off;
