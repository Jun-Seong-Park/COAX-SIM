function [Acc_d, Acc_Max, Thrust_d, Vel_d] = POS2THR(pos, vel, pos_d, angle_max, d_hat_f, p)
% POS2THR  PD position controller + disturbance compensation -> desired acceleration & thrust
%   [Acc_d, Acc_Max, Thrust_d, Vel_d] = POS2THR(pos, vel, pos_d, angle_max, d_hat_f, p)
%
%   PD control with force disturbance compensation:
%     e     = pos - pos_d
%     Acc_d = -Kp * e - Kd * vel - d_hat_f / Mass
%
%   Inputs:
%     pos       [3x1] : current position (NED)
%     vel       [3x1] : current velocity (NED)
%     pos_d     [3x1] : desired position (NED)
%     angle_max [3x1] : max attitude angles
%     d_hat_f   [3x1] : estimated force disturbance (NED) [N]
%     p         struct: params (fields: g, Mass, K1_pos, K2_pos)
%
%   Outputs:
%     Acc_d    [3x1] : desired acceleration (NED, saturated)
%     Acc_Max  [3x1] : acceleration limits
%     Thrust_d scalar: desired total thrust
%     Vel_d    [3x1] : virtual velocity command (for plotting)

    Kp = diag(p.K1_pos);
    Kd = diag(p.K2_pos);

    att_max   = angle_max(1);
    acc_z_max = 9.5;
    acc_z_min = -3 * p.g;

    % Position error
    e = pos - pos_d;

    % PD control law with disturbance compensation
    Acc_d = -Kp * e - Kd * vel - d_hat_f / p.Mass;
    Vel_d = -Kp / Kd * e;              % equilibrium velocity (for plotting)

    % Saturation
    acc_z_d    = min(max(Acc_d(3), acc_z_min), acc_z_max);
    acc_xy_max = abs(acc_z_d - p.g) * tan(att_max);
    acc_x_d    = min(max(Acc_d(1), -acc_xy_max), acc_xy_max);
    acc_y_d    = min(max(Acc_d(2), -acc_xy_max), acc_xy_max);

    Acc_Max = [acc_xy_max; acc_xy_max; acc_z_max];
    Acc_d   = [acc_x_d; acc_y_d; acc_z_d];

    Thrust_d = -norm(Acc_d - [0;0;p.g]) * p.Mass;
end
