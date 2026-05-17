function Torque = BACKDOB(state, state_1dot, Desired, U_max, d_hat, K1_vec, K2_vec, p)
% BACKDOB  Backstepping controller with disturbance observer compensation
%   Torque = BACKDOB(state, state_1dot, Desired, U_max, d_hat, K1_vec, K2_vec, p)
%
%   Inputs:
%     state      [3x1] : attitude angles [phi; theta; psi]
%     state_1dot [3x1] : attitude rates  [phi_dot; theta_dot; psi_dot]
%     Desired    [3x3] : desired trajectory [Theta_d, Theta_d_dot, Theta_d_ddot]
%     U_max      [3x1] : torque saturation limits
%     d_hat      [3x1] : estimated disturbance torque (from observer)
%     K1_vec     [3x1] : backstepping gain vector (first layer)
%     K2_vec     [3x1] : backstepping gain vector (second layer)
%     p          struct: params struct (uses p.Inertia [3x3])
%
%   Outputs:
%     Torque     [3x1] : commanded control torque (saturated)

    K1 = diag(K1_vec);
    K2 = diag(K2_vec);

    Theta     = state(1:3);
    Theta_dot = state_1dot(1:3);

    Theta_d      = Desired(:,1);
    Theta_d_dot  = Desired(:,2);
    Theta_d_ddot = Desired(:,3);

    % Error variables
    Z1     = Theta - Theta_d;
    Z1_dot = Theta_dot - Theta_d_dot;

    % Virtual control and second error
    theta_2d    = Theta_d_dot - K1 * Z1;
    Z2          = Theta_dot - theta_2d;
    Theta_2d_dd = Theta_d_ddot - K1 * Z1_dot;

    % Backstepping control law with disturbance compensation
    Torque = -p.Inertia * (Z1 - Theta_2d_dd + K2 * Z2) - d_hat;

    % Saturation
    Torque = min(max(Torque, -U_max), U_max);
end
