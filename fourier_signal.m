%loading data
f_k = load('data.txt');
disp(f_k)

% Total number of data points in f_k. Length of the vector
M = length(f_k);

stride_points = 150;
sampling_freq = 100; % from readme file
delta_t = 1/ sampling_freq; % time between each sample
T = stride_points * delta_t; % period of the signal, time for one full stride
t = (0:M-1)' * delta_t; % time row vector spanning entire length of the data

%% Fourier Series Coefficients Calculation
N=7;  % number of fourier coefficients (Try higher if needed)

a0 = mean(f_k); % DC component, represents average value of signal over time.

% arrays initialized to store Fourier coefficients
a_n = zeros(1,N); 
b_n = zeros(1,N);
omega_n = 2*pi/T; % Corrected angular frequency calculation

for n = 1:N
    a_n(n) = (2/M) * sum(f_k .* cos(omega_n * n * t));  % For cosine terms
    b_n(n) = (2/M) * sum(f_k .* sin(omega_n * n * t));  % For sine terms
end

disp('Fourier Coefficients:');
disp(['a_0 = ', num2str(a0)]);
disp(['a_n = ', num2str(a_n)]);
disp(['b_n = ', num2str(b_n)]);

% Reconstruct the signal using the Fourier coefficients
f_reconstruct = a0;
for n = 1:N
    f_reconstruct = f_reconstruct + a_n(n) * cos(omega_n * n * t) + b_n(n) * sin(omega_n * n * t);
end

figure;
plot(t, f_k, 'r--', 'Linewidth', 2, 'Marker', 'o', 'MarkerSize', 6); % Purple original line with circle markers
hold on;

plot(t, f_reconstruct, 'b--', 'Linewidth', 1.5, 'Marker', 'd', 'MarkerSize', 4); % Blue fitted line with diamond markers
legend('Original Data', 'Fourier Fitting');
xlabel('Time(seconds)');
ylabel('Ankle Joint Angle(degrees)');
title('Original vs Fourier fit');
grid on;