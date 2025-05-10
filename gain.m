clear();
%%% System Parameters %%%
R = 0.5;
L = 1e-6;
Ts = 0.00005;
wc = pi/10;
%%%
s = tf('s');
z = tf('z', Ts);

% Continuous-time system
sys = 1/(L*s + R);
sys_d = c2d(sys, Ts); % Discretized system

% PI Controller
ki = 1 - exp(-R*Ts/L);
k = R * (wc / (1 - exp(-R*Ts/L)));
controller = k * (1 + ki/(z-1));

% Open-loop system
fp = controller * sys_d; % Use multiplication instead of series()
cl = feedback(fp, 1); % Closed-loop system

%%% Bode plots %%%
figure;
hold on;
bode(controller);
bode(sys_d);
margin(fp);
legend('Controller', 'Plant', 'Return Ratio');
grid on;

%%% Closed-loop step response %%%
figure;
step(cl);
grid on;
