clc; clear; close all;

% Given Parameters
Kvco = 2 * pi * 180e6; 
M = 16;
Icp = 7.43e-3;
R1 = 610; 
C1 = 129.3e-12; 
C2 = C1 / 12.93;

% Compute Constant
const = Icp * Kvco / (2 * pi * M * (C1 + C2));

% Define Loop Gain Transfer Function
LG = tf([const * R1 * C1, const], [R1 * C1 * C2 / (C1 + C2), 1, 0, 0]);

% Compute Reference Noise Transfer Function
NTF_ref = M * feedback(LG, 1);

% Compute Icp Noise Transfer Function
const2 = 2 * pi * M / Icp;
NTF_Icp = const2 * feedback(LG, 1);

% Display Transfer Functions
disp('Reference Noise Transfer Function H_ref(s):');
tf(NTF_ref)  % This explicitly shows the transfer function

disp('Icp Noise Transfer Function H_Icp(s):');
tf(NTF_Icp)  % This explicitly shows the transfer function

NTF_div = NTF_ref;
disp('Icp Noise Transfer Function H_div(s):');
tf(NTF_div)  % This explicitly shows the transfer function

NTF_vco = 1/(1+LG);
disp('Icp Noise Transfer Function H_vco(s):');
tf(NTF_vco)  % This explicitly shows the transfer function
