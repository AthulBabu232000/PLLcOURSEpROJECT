clc; clear; close all;

% Given Parameters
Kvco = 2 * pi * 180e6; 
M = 16;
Icp = 7.43e-3;
R1 = 610; 
C1 = 129.3e-12; 
C2 = C1 / 12.93;

% Define Laplace variable s
s = tf('s'); 

% Loop Filter Transfer Function
Fs = tf([R1*C1 1], [R1*C1*C2 C1+C2 0]);

% Open Loop Gain
LG = (Icp * Kvco * Fs) / (2 * pi * M * s);

% Closed-Loop Noise Transfer Functions
NTF = M * feedback(LG, 1); % For reference and divider noise
NTF_LPF = (Kvco/s)*(1/(1+LG)); % For loop filter noise
NTF_VCO = (1/(1+LG));          % For VCO noise

% Frequency vector (rad/s)
w = logspace(4, 9, 1000); % 10 kHz to 1 GHz

% Compute magnitude response
[mag, ~, wout] = bode(NTF, w);
mag = squeeze(mag); 
mag_dB = 20*log10(mag);

% Find max magnitude (usually 0 dB at low frequencies)
max_dB = mag_dB(1);

% Find the 3 dB drop point
bw_idx = find(mag_dB <= max_dB - 3, 1); 

if ~isempty(bw_idx)
    pll_bw = wout(bw_idx) / (2*pi); % Convert from rad/s to Hz
    fprintf('PLL Bandwidth ≈ %.2f Hz\n', pll_bw);
else
    fprintf('3 dB bandwidth not found in specified frequency range.\n');
end

% Optional: Plot magnitude response
figure;
semilogx(wout/(2*pi), mag_dB, 'LineWidth', 2);
xlabel('Frequency (Hz)');
ylabel('Magnitude (dB)');
title('Magnitude Response of NTF');
grid on;
