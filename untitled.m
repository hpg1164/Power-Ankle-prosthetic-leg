% Load the data from the files
% gait_data = load('data.txt'); % Load gait data
robot_data = load('data_2.txt'); % Load robot data


% Create time vectors for each dataset
%  time_gait = linspace(0, 1, length(gait_data)); % Time vector for gait data
time_robot = linspace(0, 1, length(robot_data)); % Time vector for robot data

% Plot the two datasets
% figure;
%  plot(time_gait, gait_data, 'b-', 'LineWidth', 2); % Plot gait data (blue solid line)
hold on;
plot(time_robot, robot_data, 'r-', 'LineWidth', 2); % Plot robot data (red solid line)

% Add labels, title, and legend
title('PAFP: Ankle Angle' ,'FontSize',27);
xlabel('% Gait Cycle','FontSize',27);
ylabel('Angle(Degrees)','FontSize',	27);
legend('Reference Gait','PAFP','FontSize',25);
legend('PAFP','FontSize',25);

grid on;
hold off;