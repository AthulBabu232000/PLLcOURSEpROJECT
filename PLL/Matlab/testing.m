clc; clear; close all;

% Parameters
Icp = 100e-6;              % Charge pump current (100 µA)
R = 1e3;                   % Resistor (1 kΩ)
C1 = 4.7e-9;               % Capacitor 1 (4.7 nF)
C2 = 0.36e-9;              % Capacitor 2 (0.36 nF)

% Simulation parameters
fs = 1e9;                   % Sampling frequency
T = 1e-7;                   % Simulation time
dt = 1/fs;                   % Time step
t = 0:dt:T;                  % Time vector

% PFD signals
ref_clk = square(2 * pi * 113e6 * t);   % Reference clock (113 MHz)
fb_clk = square(2 * pi * 1.8e9 * t);    % Feedback clock (1.8 GHz)

% UP and DOWN signals (from PFD)
UP = ref_clk > fb_clk;   % UP signal (charge)
DOWN = fb_clk > ref_clk; % DOWN signal (discharge)

% Charge pump output current
I_out = Icp * (UP - DOWN);

% Loop filter response (RC network)
Vc = zeros(1, length(t));  % Initialize control voltage
for i = 2:length(t)
    dVc = (I_out(i) / C1) * dt - (Vc(i-1) / (R * C1)) * dt;  % RC filter equation
    Vc(i) = Vc(i-1) + dVc; 
end

% Plotting
figure;

% PFD signals
subplot(3, 1, 1);
plot(t, UP, 'r', 'LineWidth', 2);
hold on;
plot(t, DOWN, 'b', 'LineWidth', 2);
legend('UP', 'DOWN');
title('PFD Signals');
xlabel('Time (s)');
ylabel('Logic Levels');
grid on;

% Charge pump current output
subplot(3, 1, 2);
plot(t, I_out, 'k', 'LineWidth', 2);
title('Charge Pump Output Current');
xlabel('Time (s)');
ylabel('Current (A)');
grid on;

% Control voltage
subplot(3, 1, 3);
plot(t, Vc, 'g', 'LineWidth', 2);
title('Charge Pump Output Voltage (VCO Control Voltage)');
xlabel('Time (s)');
ylabel('Voltage (V)');
grid on;
