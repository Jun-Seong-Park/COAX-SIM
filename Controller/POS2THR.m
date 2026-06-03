function [acc_d, acc_max, total_thrust_d, vel_d] = POS2THR(pos, vel, Desired_pos, d_hat_a, p)
% POS2THR  Backstepping position controller (+DOB) -> desired acceleration & thrust
%   [acc_d, acc_max, total_thrust_d, vel_d] = POS2THR(pos, vel, Desired_pos, d_hat_a, p)
%
%   Outer-loop position backstepping (ported from the validated 1-axis
%   reference in test.m), extended to 3 NED axes, with force-disturbance
%   compensation and drone-specific (tilt-limited) acceleration saturation.
%
%   Backstepping law (per axis):
%     z1        = pos - pos_d
%     vel_d     = pd_dot  - K1*z1                 (virtual velocity cmd, saturated)
%     z2        = vel - vel_d
%     vel_d_dot = pd_ddot - K1*(vel - pd_dot)
%     a_ideal   = vel_d_dot - K2*z2 - z1          (ideal accel, pre-compensation)
%     acc_d     = a_ideal - d_hat_a               (DOB compensation, accel-level)
%
%   Inputs:
%     pos         [3x1] : current position (NED)
%     vel         [3x1] : current velocity (NED)
%     Desired_pos [3x3] : [pos_d, pd_dot, pd_ddot] from CmdFilterPos
%     d_hat_a     [3x1] : estimated force disturbance as acceleration (NED) [m/s^2]
%                         (= d_hat_f / Mass; same level as test.m's d_hat)
%     p           struct: fields g, Mass, K1_pos, K2_pos, vel_max, angle_max
%
%   Outputs:
%     acc_d          [3x1] : desired acceleration (NED, saturated)
%     acc_max        [3x1] : acceleration limits [acc_xy_max; acc_xy_max; acc_z_max]
%     total_thrust_d scalar: desired total thrust (negative, NED-down)
%     vel_d          [3x1] : virtual velocity command (for plotting)

    % --- Desired position and its derivatives (from command filter) ---
    pos_d   = Desired_pos(:,1);
    pos_d_dot  = Desired_pos(:,2);
    pos_d_ddot = Desired_pos(:,3);

    % --- Backstepping gains ---
    K1 = diag([5; 5; 5]);
    K2 = diag([3; 3; 3]);

    % --- Backstepping control law (test.m, vectorized to 3 axes) ---
    z1    = pos - pos_d;                          % position error
    vel_d = pos_d_dot - K1 * z1;                     % virtual velocity command
    vel_d = min(max(vel_d, -p.vel_max), p.vel_max);   % saturate virtual velocity

    z2        = vel - vel_d;                      % velocity error
    vel_d_dot = pos_d_ddot - K1 * (vel - pos_d_dot);    % derivative of virtual command
    a_ideal   = vel_d_dot - K2 * z2 - z1;         % ideal accel (pre-compensation)

    acc_d = a_ideal - d_hat_a;                    % DOB compensation (accel-level)

    % --- Drone acceleration saturation (tilt-limited) ---
    tilt_angle_max = p.angle_max(1);              % roll/pitch max [rad] (positive)
    acc_z_max      = 9.5;                          % descend accel max (< g)
    acc_z_min      = -3 * p.g;                     % ascend accel max

    acc_z_d    = min(max(acc_d(3), acc_z_min), acc_z_max);     % z first
    acc_xy_max = abs(acc_z_d - p.g) * tan(tilt_angle_max);     % xy limit from tilt
    acc_x_d    = min(max(acc_d(1), -acc_xy_max), acc_xy_max);
    acc_y_d    = min(max(acc_d(2), -acc_xy_max), acc_xy_max);

    acc_max = [acc_xy_max; acc_xy_max; acc_z_max];
    acc_d   = [acc_x_d; acc_y_d; acc_z_d];

    % --- Total thrust from desired acceleration (gravity removed) ---
    total_thrust_d = -norm(acc_d - [0;0;p.g]) * p.Mass;   % negative (NED-down)
end
