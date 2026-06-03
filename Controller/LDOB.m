function [d_hat, z] = LDOB(omega, Torque_d, z, p)
% LDOB  Linear Disturbance Observer
%   [d_hat, z] = LDOB(omega, Torque_d, z, p)
%
%   Transfer function: d_hat/d = L / (s + L)
%
%   Inputs:
%     omega    [3x1] : angular velocity [p; q; r]
%     Torque_d [3x1] : applied control torque
%     z        [3x1] : observer internal state
%     p        struct: params (uses p.Inertia, p.dt_ctrl, p.L_dob_ang)
%
%   Outputs:
%     d_hat    [3x1] : estimated disturbance torque
%     z        [3x1] : updated internal state

    L = p.L_dob_ang;

    % Auxiliary variable
    p_val = L * p.Inertia * omega;

    % Disturbance estimate
    d_hat = z + p_val;

    % Observer dynamics (Euler integration)
    z_dot = -L * (d_hat + Torque_d);
    z     = z + z_dot * p.dt_ctrl;
end
