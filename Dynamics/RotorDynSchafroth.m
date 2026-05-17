function X_rot = RotorDynSchafroth(X_rot, X_cmd, dt, p)
% RotorDynSchafroth  Rotor dynamics based on Schafroth 2010
%   X_rot = RotorDynSchafroth(X_rot, X_cmd, dt, p)
%
%   Model: J_rot * Omega_dot = K_motor*(Omega_cmd - Omega)
%                              - gamma*Omega^2 + gamma*Omega_cmd^2
%
%   First term:  motor torque (linear, ESC + BLDC)
%   Second term: aerodynamic drag torque (nonlinear, proportional to Omega^2)
%   Third term:  steady-state aero torque at commanded speed
%
%   In steady state: K_motor*(Omega_cmd - Omega_ss) = gamma*(Omega_ss^2 - Omega_cmd^2)
%                    => Omega_ss = Omega_cmd  (equilibrium)
%
%   Reference:
%     Schafroth 2010, "Modeling, system identification and robust control
%     of a coaxial micro helicopter", Control Engineering Practice.
%
%   Inputs:
%     X_rot  [2x1] : current [Omega_up; Omega_dw] [rad/s]
%     X_cmd  [2x1] : commanded [Omega_up_d; Omega_dw_d] [rad/s]
%     dt            : integration step [s]
%     p      struct : params (uses p.J_rot, p.K_motor, p.gamma)
%
%   Output:
%     X_rot  [2x1] : updated [Omega_up; Omega_dw] [rad/s]

    for k = 1:2
        Q_motor = p.K_motor * (X_cmd(k) - X_rot(k));
        Q_aero  = p.gamma(k) * (X_rot(k)^2 - X_cmd(k)^2);

        Omega_dot  = (Q_motor - Q_aero) / p.J_rot;
        X_rot(k)   = max(X_rot(k) + Omega_dot * dt, 0);
    end
end
