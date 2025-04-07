clc; close all;

Kvco = 2 * pi * 180e6; 
M = 16;
Icp = 100e-6;
R1 = 4.25e4; 
C1 = 1.237e-12; 
C2 = 0.0095e-12;
R3 = 5000;
C3 = 0.95e-12;

% Calculate Frequencies
wz = 1 / (R1 * C1);
disp(['Zero Frequency: ', num2str(wz)]);

w_ugb = wz * sqrt((C1 / C2) + 1);
disp(['Unity Gain Bandwidth: ', num2str(w_ugb)]);

wp3 = (C1 + C2) / (R1 * C1 * C2);
wp4 = 1 / (R3 * C3);
disp(['Pole Frequencies: ', num2str(wp3), ', ', num2str(wp4)]);

Const1 = (Icp * Kvco) / (2 * pi * M);

% Open Loop Transfer Function
H_open = tf([C1 * R1 * Const1, Const1], ...
            [R1 * R3 * C1 * C2 * C3, ...
             R1 * C1 * C2 + R3 * C1 * C3 + R3 * C2 * C3 + R1 * C1 * C3, ...
             C1 + C2 + C3, 0, 0]);

% Bode Plot with Combined Magnitude and Phase, with Wugb, Poles, and Zeros Marked
figure;

% Use Bode plot with both magnitude and phase
[mag, phase, w] = bode(H_open);

% Plot Magnitude
subplot(2,1,1);
semilogx(w, squeeze(20*log10(mag)), 'b', 'LineWidth', 2);
hold on;

% Mark zeros and poles
[z, p, k] = tf2zp([C1*R1*Const1, Const1], ...
                   [R1*R3*C1*C2*C3, R1*C1*C2 + R3*C1*C3 + R3*C2*C3 + R1*C1*C3, C1 + C2 + C3, 0, 0]);
scatter(abs(z), interp1(w, squeeze(20*log10(mag)), abs(z)), 'ro', 'filled', 'DisplayName', 'Zeros');
scatter(abs(p), interp1(w, squeeze(20*log10(mag)), abs(p)), 'kx', 'DisplayName', 'Poles');

% Mark Unity Gain Bandwidth (Wugb)
xline(w_ugb, '--g', 'Wugb', 'LabelHorizontalAlignment', 'left', 'LabelVerticalAlignment', 'middle', 'LineWidth', 2);

title('Open Loop Bode Plot - Magnitude');
xlabel('Frequency (rad/s)');
ylabel('Magnitude (dB)');
legend('Magnitude', 'Zeros', 'Poles', 'Wugb');
grid on;

% Plot Phase
subplot(2,1,2);
semilogx(w, squeeze(phase), 'r', 'LineWidth', 2);
hold on;

% Mark Unity Gain Bandwidth on Phase plot
xline(w_ugb, '--g', 'Wugb', 'LabelHorizontalAlignment', 'left', 'LabelVerticalAlignment', 'middle', 'LineWidth', 2);

title('Open Loop Bode Plot - Phase');
xlabel('Frequency (rad/s)');
ylabel('Phase (degrees)');
grid on;

% Display Phase Margin
[Gm, Pm, Wgm, Wpm] = margin(H_open);
disp(['Phase Margin: ', num2str(Pm)]);

% Closed-Loop Transfer Function
H_closed = feedback(H_open, 1/M);

figure;
bode(H_closed);
hold on;
grid on;
title('Closed Loop Transfer Function');
set(findall(gcf, 'type', 'line'), 'linewidth', 2);

% Compute Step Response and Settling Time
figure;
step(H_closed);
title('Step Response of Closed-Loop System');
grid on;

% Analyze Step Response Characteristics
info = stepinfo(H_closed);
T_s = info.SettlingTime;
disp(info);
disp(['Settling Time (2% criterion): ', num2str(T_s), ' seconds']);
