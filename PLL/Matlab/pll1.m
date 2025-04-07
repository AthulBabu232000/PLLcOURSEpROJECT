clear;
clc;

%% Parameters
f_ref = 10e6;           % Reference frequency (10 MHz)
K_vco = 2 * pi * 10e6;  % VCO sensitivity (rad/V)
f_center = 100e6;       % VCO center frequency (100 MHz)
N = 10;                 % Frequency divider factor
dt = 1e-9;              % Time step (1 ns)
Tsim = 1e-3;            % Total simulation time (1 ms)

%% Loop Filter Parameters (Design for 10 kHz bandwidth)
R1 = 1e3;               % Ohms
C1 = 1e-9;              % Farads
C2 = 10e-9;             % Farads

%% Transfer Function of Loop Filter
s = tf('s');
H_loop = (R1 * C2 * s + 1) / (R1 * C1 * C2 * s^2 + (C1 + C2) * s);

%% Initialize Signals
time = 0:dt:Tsim;
vco_freq = zeros(size(time));
vco_output = zeros(size(time));
phase_error = zeros(size(time));
control_voltage = zeros(size(time));
ref_phase = zeros(size(time));
vco_phase = zeros(size(time));

%% Simulation Loop
for k = 2:length(time)
    disp("I am still in the loop");
    % Reference phase
    ref_phase(k) = ref_phase(k-1) + 2 * pi * f_ref * dt;
    
    % Phase detector output (phase error)
    phase_error(k) = mod(ref_phase(k) - vco_phase(k-1), 2 * pi);
    
    % Loop filter output (step-by-step computation)
    [y, ~, x] = lsim(H_loop, phase_error(1:k), time(1:k));
    control_voltage(k) = y(end);  % Take the last computed value
    
    % VCO frequency adjustment based on control voltage
    vco_freq(k) = f_center + (K_vco * control_voltage(k)) / (2 * pi);
    
    % VCO phase integration
    vco_phase(k) = vco_phase(k-1) + 2 * pi * vco_freq(k) * dt;
    
    % VCO output
    vco_output(k) = sin(vco_phase(k));
end

%% Plot Results
figure;
subplot(3,1,1);
plot(time, phase_error);
title('Phase Error');
xlabel('Time (s)');
ylabel('Phase Error (rad)');
grid on;

subplot(3,1,2);
plot(time, control_voltage);
title('Control Voltage');
xlabel('Time (s)');
ylabel('Voltage (V)');
grid on;

subplot(3,1,3);
plot(time, vco_freq / 1e6);
title('VCO Frequency');
xlabel('Time (s)');
ylabel('Frequency (MHz)');
grid on;

%% Determine Settling Time
tolerance = 0.01; % 1% of final value
final_value = mean(vco_freq(end-100:end));
settling_index = find(abs(vco_freq - final_value) < tolerance * final_value, 1);
settling_time = time(settling_index);
disp(['Settling time: ', num2str(settling_time * 1e6), ' μs']);

%% Lock Status Verification
if abs(final_value - f_ref * N) < tolerance * f_ref * N
    disp('PLL is in lock state.');
else
    disp('PLL failed to lock.');
end
