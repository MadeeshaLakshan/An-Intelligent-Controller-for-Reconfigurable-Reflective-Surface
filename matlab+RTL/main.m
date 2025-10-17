% ==== IRS Phase and SPI Code Generator ====
close all;
clear;

% ==== System Parameters ====
frequency = 2.4e9; % 2.4 GHz carrier frequency
c = 3e8; % Speed of light in m/s
wavelength = c / frequency;

% ==== IRS Configuration ====
Nr = 1; % 1 row
Nc = 9; % 9 columns (9 IRS elements)
unit_cell_size = 0.026; % Actual physical size of each unit cell (in meters)

% ==== Positions ====
xt = 0.9; yt = 0.9; % Transmitter position
xr = 0.5; yr = 0.9; % Receiver position
% IRS element positions
x_irs = linspace(0, (Nc - 1) * unit_cell_size, Nc);
y_irs = zeros(1, Nc); % all elements aligned along x-axis

% ==== Compute Phase Shifts ====
irs_phase_shifts = compute_phase_shifts(x_irs, y_irs, xt, yt, xr, yr, wavelength);

% ==== Estimate Voltages ====
estimated_voltages = estimate_voltages(irs_phase_shifts);

% ==== Convert Voltages to 16-bit PWM Counts ====
gain = 1+18/(3.3+0.24); % example gain (adjust as needed)
max_voltage = 3.3; % full-scale reference voltage (adjust to your system)

pwm_counts = voltages_to_pwm(estimated_voltages, gain, max_voltage);

% ==== Send PWM counts via UART ====
for k = 1:length(pwm_counts)
 uart_send_16bit_word(pwm_counts(k), 'COM5');
 pause(0.1); % short delay between sends
end

% ==== Display Results ====
disp('IRS Phase Shifts (radians):'); disp(irs_phase_shifts);
disp('IRS Phase Shifts (degrees):'); disp(rad2deg(irs_phase_shifts));
disp('Estimated Voltages:'); disp(estimated_voltages);
disp('PWM Counts (16-bit):'); disp(pwm_counts);
disp('PWM Counts (as Percentage of 16-bit max):');
pwm_percentage = (double(pwm_counts) ./ 65535) * 100;
disp(pwm_percentage);
