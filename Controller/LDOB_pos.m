function [d_hat_f, z] = LDOB_pos(vel, Acc_cmd, z, p)
% LDOB_pos  Linear Disturbance Observer for translational dynamics
%   [d_hat_f, z] = LDOB_pos(vel, Acc_cmd, z, p)
%
%   Estimates external force disturbance in NED frame.
%   Same structure as LDOB (attitude), applied to: M*a = F_cmd + d_f
%
%   Transfer function: d_hat/d = L / (s + L)
%
%   Inputs:
%     vel      [3x1] : velocity (NED) [m/s]
%     Acc_cmd  [3x1] : commanded acceleration (NED) [m/s^2]
%     z        [3x1] : observer internal state
%     p        struct: params (uses p.Mass, p.dt_ctrl, p.L_dob_pos)
%
%   Outputs:
%     d_hat_f  [3x1] : estimated force disturbance (NED) [N]
%     z        [3x1] : updated internal state

    L = p.L_dob_pos;
    M = p.Mass;

    % Auxiliary variable
    p_val = L * M * vel;

    % Disturbance estimate
    d_hat_f = z + p_val;

    % Observer dynamics (Euler integration)
    %   M*v_dot = F_cmd + d_f
    %   F_cmd = M * Acc_cmd  (commanded force)
    z_dot = -L * (d_hat_f + M * Acc_cmd);
    z     = z + z_dot * p.dt_ctrl;
end
