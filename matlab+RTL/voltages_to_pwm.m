function pwm_counts = voltages_to_pwm(estimated_voltages, gain, max_voltage)
    % voltages_to_pwm - Convert voltages to 16-bit PWM counts
    %
    % Inputs:
    %   estimated_voltages : array of estimated voltages
    %   gain              : gain factor to scale voltages
    %   max_voltage       : reference maximum voltage for full-scale PWM (e.g. 3.3V, 5V, etc.)
    %
    % Output:
    %   pwm_counts : array of 16-bit integers (0 to 65535)

    if nargin < 2
        gain = 1.0; % default unity gain if not given
    end
    if nargin < 3
        max_voltage = 3.3; % default full-scale voltage
    end

    % Apply gain correction
    scaled_voltage = estimated_voltages ./ gain;

    % Normalize to [0, 1] range
    normalized_voltage = scaled_voltage ./ max_voltage;

    % Clip values to avoid overflow
    normalized_voltage = max(0, min(1, normalized_voltage));

    % Convert to 16-bit PWM counts
    pwm_counts = uint16(round(normalized_voltage * (2^16 - 1)));
end
