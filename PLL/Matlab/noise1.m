clc; clear; close all;

% Given Parameters
Kvco = 2 * pi * 180e6; 
M = 16;
Icp = 7.43e-3;
R1 = 610; 
C1 = 129.3e-12; 
C2 = C1 / 12.93;

% Compute Constants
s = tf('s'); % Define Laplace variable s
Fs=tf([R1*C1 1],[R1*C1*C2 C1+C2 0]);
LG = (Icp * Kvco * Fs) / (2 * pi * M * (C1 + C2) * s);
NTF_ref = M * feedback(LG, 1);
NTF_div = M * feedback(LG, 1);


data = readmatrix('Ref_Noise.csv'); % Replace with actual file name
frequency = data(:,1); % First column: Frequency in Hz
noise_dBc = data(:,2); % Second column: Noise in dBc

noise_linear = 10.^(noise_dBc / 10);

% Evaluate the magnitude response of the transfer function
[mag, ~] = bode(NTF_ref, 2 * pi * frequency); % Convert Hz to rad/s
mag_linear = squeeze(mag).^2; % Square the magnitude

% Apply the transfer function to the noise
processed_noise_linear = noise_linear .* mag_linear;
processed_noise_dBc = 10 * log10(processed_noise_linear);


figure;
%semilogx(frequency, noise_dBc, 'b', 'LineWidth', 2); hold on;
semilogx(frequency, processed_noise_dBc, 'r', 'LineWidth', 2);


grid on;
xlabel('Frequency (Hz)');
ylabel('Noise (dBc)');
title('Reference Noise Spectrum');
legend('Original Noise', 'Processed Noise');

set(gca, 'FontSize', 12);
