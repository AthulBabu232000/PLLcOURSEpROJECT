clc; close all;
figure

Kvco = 2*pi*180e6; 


M = 16;
Icp = 7.43e-3;

R1 = 610; 
C1 = 129.3e-12; 
C2 = C1/12.93;

wz = 1 / (R1*C1);
disp(['Zero frequency (wz): ', num2str(wz)]);

w_ugb = wz * sqrt((C1/C2) + 1);
disp(['Unity-gain BW (w_ugb): ', num2str(w_ugb)]);

wp3 = (C1 + C2) / (R1*C1*C2);
disp(['Pole frequency (wp3): ', num2str(wp3)]);

PM_Max = (atan(w_ugb/wz) - atan(w_ugb/wp3)) * 360/(2*pi);
disp(['Maximum Phase Margin (PM_Max): ', num2str(PM_Max)]);

Const1 = (Icp * Kvco) / (2*pi*M*C1*C2*R1);
H_open = tf([C1*R1*Const1, Const1], [1, ((C1 + C2) / (C1 * C2 * R1)), 0, 0]);
H_open

figure;
[mag, phase, w] = bode(H_open);
bode(H_open);
[Gm, Pm, Wgm, Wpm] = margin(H_open);
disp(['Phase Margin: ', num2str(Pm)]);

hold on;
grid on;
title('Open loop bode plot type II order 3 (Magnitude and Phase)');
set(findall(gcf, 'type', 'line'), 'linewidth', 2);

% Find poles and zeros accurately
[b, a] = tfdata(H_open, 'v');
[z, p, k] = tf2zp(b, a);
disp('Zeros:');
disp(z);
disp('Poles:');
disp(p);

% Mark zeros in red circles and poles in black crosses on the magnitude plot
[mag, phase, w] = bode(H_open);
figure;
semilogx(w, 20*log10(squeeze(mag)), 'b', 'LineWidth', 4);
hold on;
semilogx(abs(z), interp1(w, 20*log10(squeeze(mag)), abs(z)), 'ro', 'MarkerSize', 8, 'LineWidth', 2);
semilogx(abs(p), interp1(w, 20*log10(squeeze(mag)), abs(p)), 'kx', 'MarkerSize', 8, 'LineWidth', 2);

grid on;
title('Open-loop Transfer Function Bode plot');
xlabel('Frequency (rad/s)');
ylabel('Magnitude (dB)');
legend('Magnitude Response', 'Zeros', 'Poles');

% Closed Loop Gain
H_closed = tf([(C1*R1*Const1*M), (M*Const1)], [1, ((C1 + C2) / (C1 * C2 * R1)), (C1 * R1 * Const1), Const1]);
figure;
bode(H_closed);
H_closed

disp(bandwidth(H_closed))
hold on;
grid on;
title('Open loop bode plot type II order 3 (Magnitude and Phase)');
set(findall(gcf, 'type', 'line'), 'linewidth', 4);

% Step Response Analysis
figure;
stepplot(H_closed);
title('Step Response of Closed-Loop System type II order 3');
grid on;



info = stepinfo(H_closed);
T_s = info.SettlingTime;
disp(['Settling Time (2% criterion): ', num2str(T_s), ' seconds']);