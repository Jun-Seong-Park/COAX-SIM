function [Desired, filt] = CmdFilter(Theta_d_raw, filt, dt, angle_max, angle_dot_max, Torque_max, p)
% CmdFilter  Second-order command filter with physically-determined wn
%   [Desired, filt] = CmdFilter(Theta_d_raw, filt, dt, angle_max, angle_dot_max, Torque_max, p)
%
%   wn is recomputed every step from current Torque_max.

    J     = diag(p.Inertia);
    d_max = p.Dis_max(2);

    % Use effective angle scale for wn (cap yaw at same scale as roll/pitch)
    angle_scale = min(angle_max, angle_max(1));
    wn_torque   = sqrt(max(Torque_max - d_max, 0.01) ./ (J .* angle_scale));
    wn          = 0.8 * wn_torque;     % rate limiting handled by saturation below
    zeta      = 1.0;

    Theta_f     = filt(1:3);
    Theta_f_dot = filt(4:6);

    Theta_f_ddot = -2*zeta*wn .* Theta_f_dot - wn.^2 .* (Theta_f - Theta_d_raw);

    Theta_f     = Theta_f     + dt * Theta_f_dot;
    Theta_f_dot = Theta_f_dot + dt * Theta_f_ddot;

    % Saturate
    Theta_f     = min(max(Theta_f, -angle_max), angle_max);
    Theta_f_dot = min(max(Theta_f_dot, -angle_dot_max), angle_dot_max);

    % Recompute acceleration after saturation
    Theta_f_ddot = -2*zeta*wn .* Theta_f_dot - wn.^2 .* (Theta_f - Theta_d_raw);

    filt(1:3) = Theta_f;
    filt(4:6) = Theta_f_dot;

    Desired = [Theta_f, Theta_f_dot, Theta_f_ddot];
end
