clc; clear; close all;

% Simulation parameters
fs = 10 * 1.8e9;             % Sampling frequency (10x the highest clock frequency)
t = 0:1/fs:100e-10;           % Time vector (100 ns simulation)

% VCO parameters
f_center = 1.5e9;             % Center frequency (1.8 GHz)
K_vco = 1e9;                  % VCO gain (1 GHz/V)

% Control voltage (simulated charge pump output)
Vctrl = linspace(0, 1.8, length(t));   % Vctrl varies from 0 to 1.8V

% VCO frequency modulation based on Vctrl
f_vco = f_center + K_vco * (Vctrl - 0.9);  % Centered at 0.9V (midpoint)

% Generate VCO output signal
vco_output = square(2 * pi * f_vco .* t);   % Square wave output

% Normalize to 0-1 range
vco_output(vco_output < 0) = 0;

% Plotting
figure;

% Control Voltage (Vctrl)
subplot(3, 1, 1);
plot(t, Vctrl, 'LineWidth', 2, 'Color', 'b');
title('Control Voltage (Vctrl)');
xlabel('Time (s)');
ylabel('Voltage (V)');
grid on;

% VCO Frequency Variation
subplot(3, 1, 2);
plot(t, f_vco / 1e9, 'LineWidth', 2, 'Color', 'g');
title('VCO Frequency vs Time');
xlabel('Time (s)');
ylabel('Frequency (GHz)');
grid on;

% VCO Output Signal
subplot(3, 1, 3);
stairs(t, vco_output, 'LineWidth', 2, 'Color', 'r');
title('VCO Output Signal');
xlabel('Time (s)');
ylabel('Amplitude');
ylim([-0.2, 1.2]);
grid on;

hold off;
