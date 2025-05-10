clc;
clear;
close all;

% Constants
G = 4.6;                % Gear ratio
r1 = 150;               % Link length in mm (r1 = 150mm)
r2 = 150;               % Link length in mm (r2 = 150mm)
theta_range = -25:0.1:15; % Range of theta (from -25° to 15°)

% Initialize arrays to store end-effector positions and velocities
P_x = zeros(size(theta_range)); % X position of end-effector
P_y = zeros(size(theta_range)); % Y position of end-effector
v_x = zeros(size(theta_range)); % X velocity of end-effector
v_y = zeros(size(theta_range)); % Y velocity of end-effector

% Define the foot structure (simple line representing the foot)
foot_length = 100; % Length of the foot in mm
foot_angle = 0;    % Initial angle of the foot (will be updated during simulation)

% Loop through theta values
for i = 1:length(theta_range)
    theta = deg2rad(theta_range(i)); % Convert theta to radians
    theta2 = theta / G;              % Joint 2 angle (theta2 = theta1 / G)

    % Transformation matrices (A01 and A12)
    A01 = [
        -sin(theta), -cos(theta), 0, -r1 * sin(theta);
         cos(theta), -sin(theta), 0,  r1 * cos(theta);
         0,          0,          1,  0;
         0,          0,          0,  1;
    ];

    A12 = [
        -sin(theta2), -cos(theta2), 0, -r2 * sin(theta2);
         cos(theta2), -sin(theta2), 0,  r2 * cos(theta2);
         0,          0,          1,  0;
         0,          0,          0,  1;
    ];

    % Combined transformation matrix (T02 = A01 * A12)
    T02 = A01 * A12;

    % End-effector position (P = [x; y; z])
    P = T02(1:3, 4); % Extract the position vector
    P_x(i) = P(1);   % X position
    P_y(i) = P(2);   % Y position

    % Jacobian matrix (J)
    J = [
        -r1 * cos(theta) + r2 * sin(theta) * cos(theta2) + r2 * sin(theta2) * cos(theta), ...
         r2 * sin(theta) * cos(theta2) + r2 * sin(theta2) * cos(theta);
        -r1 * sin(theta) + r2 * sin(theta) * sin(theta2) - r2 * cos(theta) * cos(theta2), ...
         r2 * sin(theta) * sin(theta2) - r2 * cos(theta) * cos(theta2);
    ];

    % Joint velocities (q = [w0; w1])
    w0 = 1; % Angular velocity of joint 1 (assumed constant)
    w1 = w0 / G; % Angular velocity of joint 2
    q = [w0; w1];

    % End-effector velocity (v = J * q)
    v = J * q;
    v_x(i) = v(1); % X velocity
    v_y(i) = v(2); % Y velocity

    % Update foot angle based on theta
    foot_angle = theta; % Foot angle is the same as theta
end

% Plot the end-effector position and foot structure during the gait cycle
figure;
hold on;
axis equal;
grid on;
xlabel('X Position (mm)');
ylabel('Y Position (mm)');
title('Foot Motion During Gait Cycle');

% Plot the end-effector path
plot(P_x, P_y, 'b-', 'LineWidth', 2);

% Plot the foot structure at specific points
for i = 1:20:length(theta_range)
    % Foot position
    foot_x = [P_x(i), P_x(i) + foot_length * cos(foot_angle)];
    foot_y = [P_y(i), P_y(i) + foot_length * sin(foot_angle)];

    % Plot the foot
    plot(foot_x, foot_y, 'r-', 'LineWidth', 2);
end

% Plot the end-effector velocity during the gait cycle
figure;
plot(theta_range, v_x, 'r-', 'LineWidth', 2);
hold on;
plot(theta_range, v_y, 'g-', 'LineWidth', 2);
xlabel('Theta (degrees)');
ylabel('Velocity (mm/s)');
title('End-Effector Velocity During Gait Cycle');
legend('V_x', 'V_y');
grid on;