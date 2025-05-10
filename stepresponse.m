% Define the time constant
T = 21.45; % seconds

% Create the transfer function
numerator = 1;
denominator = [T 1];
sys = tf(numerator, denominator);

% Plot the step response
step(sys);
title('Step Response of our  System');
xlabel('Time [s]');
ylabel('Angle (°)');
grid on;