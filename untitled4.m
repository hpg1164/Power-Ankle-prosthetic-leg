% Load the data from the files
gait_data = load('data.txt'); % Load gait data
robot_data = load('data_4.txt'); % Load robot data

% Determine the common time frame
num_points = max(length(gait_data), length(robot_data));
time_common = linspace(0, max(length(gait_data), length(robot_data)) - 1, num_points); % Common time vector from zero to end

% Interpolate data to match the common time frame
gait_data_interp = interp1(linspace(0, length(gait_data) - 1, length(gait_data)), gait_data, time_common, 'linear', 'extrap');
robot_data_interp = interp1(linspace(0, length(robot_data) - 1, length(robot_data)), robot_data, time_common, 'linear', 'extrap');

% Plot the datasets
figure;
plot(time_common, gait_data_interp, 'b-', 'LineWidth', 2); % Plot interpolated gait data (blue solid line)
hold on;
plot(time_common, robot_data_interp, 'r-', 'LineWidth', 2); % Plot interpolated robot data (red solid line)

% Add labels, title, and legend
title('Ankle Angle vs Time','FontSize',27);
xlabel('Time (Frames)','FontSize',30);
ylabel('Angle (Degrees)','FontSize',30);
legend('Reference Gait','PAFP','FontSize',25);

grid on;
hold off;
