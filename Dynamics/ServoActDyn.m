function X_sw = ServoActDyn(X_sw, X_cmd, tau_sw, dt)
% ServoActDyn  First-order lag for swashplate angles (alpha, beta)
%   X_sw = ServoActDyn(X_sw, X_cmd, tau_sw, dt)
%
%   Inputs:
%     X_sw   [2x1] : current [alpha; beta] [rad]
%     X_cmd  [2x1] : commanded [alpha_d; beta_d] [rad]
%     tau_sw        : servo time constant [s]
%     dt            : integration step [s]

    X_sw = X_sw + (X_cmd - X_sw) * dt / tau_sw;
end
