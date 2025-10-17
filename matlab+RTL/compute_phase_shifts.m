function irs_phase_shifts = compute_phase_shifts(x_irs, y_irs, xt, yt, xr, yr, wavelength)
    % Compute distance from transmitter to each IRS element
    dt_irs = sqrt((x_irs - xt).^2 + (y_irs - yt).^2);
    
    % Compute distance from each IRS element to receiver
    dirs_r = sqrt((xr - x_irs).^2 + (yr - y_irs).^2);
    
    % Total path length
    total_phase = 2 * pi * (dt_irs + dirs_r) / wavelength;
    
    % === Shortest path phase shift ===
    % Wrap total phase into [-pi, pi] to ensure phase shifts
    % do not exceed ±180 degrees
    irs_phase_shifts = (mod(total_phase + pi, 2*pi) - pi);
end
