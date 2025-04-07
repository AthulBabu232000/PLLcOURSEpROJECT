clc; close all;

% System Parameters
Kvco = 2 * pi * 180e6;  % VCO gain in rad/s/V
M = 16;                 % Division ratio
Icp = 7.3e-3;           % Charge pump current in A
R1 = 610;               % Loop filter resistor in ohms
C1 = 129.3e-12;           % Loop filter capacitor in F
C2 = 10e-12;           
R3 = 16.16e3;           % Second resistor in loop filter
C3 = 0.1e-12;           
Const1 = (Icp * Kvco) / (2 * pi * M);  % Loop gain constant
Const1 = (Icp * Kvco) / (2*pi*M*C1*C2*R1);

% Original Transfer Function Coefficients
b = [C1 * R1 * Const1, Const1];  % Numerator
a = [R1 * R3 * C1 * C2 * C3, ...
     R1 * C1 * C2 + R3 * C1 * C3 + R3 * C2 * C3 + R1 * C1 * C3, ...
     C1 + C2 + C3, ...
     0, ...
     0];                         % Denominator
a= [1, ((C1 + C2) / (C1 * C2 * R1)), 0, 0]
% Remove zero at origin (leading zero in numerator)
if abs(b(1)) < 1e-12
    b = b(2:end);
end

% Create Transfer Function
H = tf(b, a);

% Display
disp('Transfer Function with Zero at Origin Removed:');
H

% Pole-Zero Map
figure;
pzmap(H);
grid on;
title('Pole-Zero Map (Zero at Origin Removed)');

% Gain and Phase Margin
[GM, PM, Wcg, Wcp] = margin(H);

% Bode plot with margin lines
figure;
[mag, phase, w] = bode(H);
mag = squeeze(mag);
phase = squeeze(phase);

% Plot magnitude and phase
yyaxis left
semilogx(w, 20*log10(mag), 'b', 'LineWidth', 2);
ylabel('Magnitude (dB)');
yline(0, '--k', 'LineWidth', 1);

yyaxis right
semilogx(w, phase, 'r', 'LineWidth', 2);
ylabel('Phase (deg)');
yline(-180, '--k', 'LineWidth', 1);

% Vertical lines for crossover frequencies
xline(Wcp, '--b', sprintf('PM = %.1f°', PM), 'LineWidth', 1.5, ...
    'LabelVerticalAlignment', 'bottom', 'LabelHorizontalAlignment', 'center');
xline(Wcg, '--r', sprintf('GM = %.1f dB', 20*log10(GM)), 'LineWidth', 1.5, ...
    'LabelVerticalAlignment', 'top', 'LabelHorizontalAlignment', 'center');

grid on;
title('Bode Plot with Phase and Gain Margin type 2 order 4 PLL');
xlabel('Frequency (rad/s)');
