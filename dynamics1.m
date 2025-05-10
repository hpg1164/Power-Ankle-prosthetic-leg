% Define the system parameters
J_c = 0.0037;  
m = 0.9715;    
d = 0.246;    
k = 0.5;    
g = 9.81;   
T_d = 1.0;  

% Initial conditions
theta_0 = -5 * pi / 180;  % Convert -5 degrees to radians
omega_0 = 0.0;  % Initial angular velocity (radians/second)
y0 = [theta_0; omega_0];

% Time span for the simulation
t_span = [0, 0.35];  % Simulate from t = 0 to t = 0.35 seconds

% Solve the system using ode45
[t, y] = ode45(@system, t_span, y0);

% Extract the results
theta = y(:, 1) * (180 / pi);  % Convert theta to degrees
time = t;  % Time points

% Plot the results
figure;
plot(time, theta, 'b', 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('Angle [°]');
title('a) Angular Position');
grid on;
xlim([0 0.35]);  % Set x-axis limits to match the example
ylim([-30 5]);   % Set y-axis limits to match the example
set(gca, 'FontSize', 12);  % Adjust font size for readability

% Define the system of ODEs as a function (Moved to the end)
function dydt = system(t, y)
    J_c = 0.1;  
    m = 1.0;    
    d = 0.1;    
    k = 0.5;    
    g = 9.81;   
    T_d = 1.0;  

    theta = y(1);  % Angle (theta)
    omega = y(2);  % Angular velocity (omega)
    
    dtheta_dt = omega;
    domega_dt = (T_d - k * omega - m * g * d * sin(theta)) / (J_c + m * d^2);
    
    dydt = [dtheta_dt; domega_dt];
end
