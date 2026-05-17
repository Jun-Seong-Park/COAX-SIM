function X_rot = RotorActDyn(X_rot, X_cmd, tau_rot, dt)
% RotorActDyn  First-order lag for rotor speeds (Omega_up, Omega_dw)
%   X_rot = RotorActDyn(X_rot, X_cmd, tau_rot, dt)
%
%   Inputs:
%     X_rot  [2x1] : current [Omega_up; Omega_dw] [rad/s]
%     X_cmd  [2x1] : commanded [Omega_up_d; Omega_dw_d] [rad/s]
%     tau_rot       : motor time constant [s]
%     dt            : integration step [s]

    X_rot = X_rot + (X_cmd - X_rot) * dt / tau_rot;
end
