clc; clear; close all;

% Simulation parameters
fs = 10 * 1.8e9;                   % Sampling frequency (10x the highest clock frequency)
t = 0:1/fs:1e-6;                    % Time vector (1 µs simulation)

% VCO parameters
f_center = 1.8e9;                   % Center frequency (1.8 GHz)
K_vco = 1e9;                        % VCO gain (1 GHz/V)
V_mid = 0.9;                        % Midpoint voltage (center frequency)

% Reference and feedback clocks
ref_clk = square(2 * pi * 113e6 * t);   % 113 MHz reference clock
ref_clk(ref_clk < 0) = 0;               % Make clock 0-1

% Control voltage initialization
Vctrl = zeros(size(t));

% D inputs (both always HIGH)
d1 = ones(size(t));
d2 = ones(size(t));

% Initialize outputs
q1 = zeros(size(t));
q2 = zeros(size(t));

% Charge pump current
Icp = 11.49e-3;                        % 11.49 mA charge pump current

% Preallocate for charge pump current output
Iout = zeros(size(t));
f_vco = zeros(size(t));
fb_clk = zeros(size(t));

% Loop filter transfer function
% Updated loop filter transfer function
num = [3.778e4, 1];                  % Numerator coefficients
den = [3.778e-9, 1, 0, 0];           % Denominator with extra 's' term
Z = tf(num, den);                     % Updated transfer function
 

% D Flip-Flop Logic Simulation with AND gate reset + dynamic VCO update
for i = 2:length(t)
    
    % Flip-flop 1 (113 MHz reference clock)
    if ref_clk(i) == 1 && ref_clk(i-1) == 0  % Rising edge detection
        q1(i) = d1(i);
    else
        q1(i) = q1(i-1);
    end
    
    % Flip-flop 2 (feedback clock)
    if fb_clk(i-1) == 0 && fb_clk(i) == 1  % Rising edge detection
        q2(i) = d2(i);
    else
        q2(i) = q2(i-1);
    end
    
    % AND gate reset condition
    if q1(i) == 1 && q2(i) == 1
        q1(i) = 0;
        q2(i) = 0;
    end
    
    % Charge pump output current
    Iout(i) = Icp * (q1(i) - q2(i));
    
    % Loop filter output voltage (control voltage)
    % Loop filter output voltage (control voltage)
    [~, Vout] = lsim(Z, Iout(1:i), t(1:i));   % Extract the output
    Vctrl(i) = Vout(end);                      % Assign only the last value

    
    % VCO frequency dynamically changes with control voltage
    f_vco(i) = (f_center + K_vco * (Vctrl(i) - V_mid)) / 16;
    
    % Generate dynamic VCO output signal
    fb_clk(i) = square(2 * pi * f_vco(i) * t(i));
end

% Normalize VCO output
fb_clk(fb_clk < 0) = 0;

% Plotting
figure;

% Control Voltage (Vctrl)
subplot(3, 1, 1);
plot(t, Vctrl, 'g', 'LineWidth', 2);
title('Control Voltage (VCO Input)');
xlabel('Time (s)');
ylabel('Voltage (V)');
grid on;

% VCO Frequency Variation
subplot(3, 1, 2);
plot(t, f_vco / 1e9, 'b', 'LineWidth', 2);
title('VCO Frequency vs Time');
xlabel('Time (s)');
ylabel('Frequency (GHz)');
grid on;

% VCO Output Signal
subplot(3, 1, 3);
stairs(t, fb_clk, 'LineWidth', 2, 'Color', 'r');
title('VCO Output Signal');
xlabel('Time (s)');
ylabel('Amplitude');
ylim([-0.2, 1.2]);
grid on;

hold off;
