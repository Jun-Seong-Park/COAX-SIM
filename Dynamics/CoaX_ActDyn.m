function X_act = CoaX_ActDyn(X_act, X_cmd, tau_sw, tau_rot, dt)
% CoaX_ActDyn  First-order actuator dynamics for coaxial rotor drone
%
%   X_act = CoaX_ActDyn(X_act, X_cmd, tau_sw, tau_rot, dt)
%
%   Inputs:
%       X_act   [4x1] : current actuator states [alpha; beta; Omega_up; Omega_dw]
%       X_cmd   [4x1] : commanded values [alpha_d; beta_d; Omega_up_d; Omega_dw_d]
%       tau_sw         : swashplate servo time constant [s]
%       tau_rot        : rotor motor time constant [s]
%       dt             : integration time step [s]
%
%   Output:
%       X_act   [4x1] : updated actuator states
%
%   Reference:
%       Schafroth 2010, "Modeling, system identification and robust control
%       of a coaxial micro helicopter", Control Engineering Practice.
%       Swashplate and rotor dynamics modeled as first-order lag:
%           x_dot = (1/tau) * (x_cmd - x)

% Swashplate angles (alpha, beta)
X_act(1:2) = X_act(1:2) + (X_cmd(1:2) - X_act(1:2)) * dt / tau_sw;

% Rotor speeds (Omega_up, Omega_dw)
X_act(3:4) = X_act(3:4) + (X_cmd(3:4) - X_act(3:4)) * dt / tau_rot;

end
