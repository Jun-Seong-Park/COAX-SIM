function [Torque_d, Torque_nom, Torque_lim] = BACKHOCBFQPDOB(state, state_1dot, Desired, state_max, d_hat, p)
% BACKHOCBFQPDOB  Backstepping + 1st-order filter + HOCBF-QP + DOB compensation
%   [Torque_d, Torque_nom, Torque_lim] = BACKHOCBFQPDOB(state, state_1dot, Desired, state_max, d_hat, p)

    persistent Torque_filt_prev Torque_prev
    if isempty(Torque_filt_prev)
        Torque_filt_prev = zeros(3,1);
        Torque_prev      = zeros(3,1);
    end

    K1 = diag(p.K1_bsc);
    K2 = diag(p.K2_bsc);
    P1 = p.P1_cbf * eye(3);
    P2 = p.P2_cbf * eye(3);
    P3 = p.P3_cbf * eye(3);

    Theta     = state(1:3);
    Theta_dot = state_1dot(1:3);

    Theta_d      = Desired(:,1);
    Theta_d_dot  = Desired(:,2);
    Theta_d_ddot = Desired(:,3);

    Theta_max     = state_max(:,1);
    Theta_dot_max = state_max(:,2);
    Torque_max    = state_max(:,3);

    % Backstepping errors
    Z1     = Theta - Theta_d;
    Z1_dot = Theta_dot - Theta_d_dot;

    theta_2d    = Theta_d_dot - K1 * Z1;
    Z2          = Theta_dot - theta_2d;
    Theta_2d_dd = Theta_d_ddot - K1 * Z1_dot;

    % Nominal torque with disturbance compensation
    Torque_nom = -p.Inertia * (Z1 - Theta_2d_dd + K2 * Z2) - d_hat;

    % 1st-order low-pass filter
    Torque_filt_prev = Torque_filt_prev + p.dt_ctrl * (Torque_nom - Torque_filt_prev) / p.tau_filt;

    % --- HOCBF constraints (with disturbance) ---
    B1 = Theta_max.^2 - Theta.^2;
    B2 = Theta_dot_max.^2 - Theta_dot.^2;

    A1_cond = 2 * Theta;
    A2_cond = 2 * Theta_dot;
    B1_cond = p.Inertia * (-2 * Theta_dot.^2 + (P1 + P2) * (-2 * Theta .* Theta_dot) + P1 * P2 * B1) ...
              - 2 * Theta .* d_hat;
    B2_cond = p.Inertia * P3 * B2 - 2 * Theta_dot .* d_hat;

    % Exclude psi from B1 (yaw angle barrier not physical with large theta_max)
    A1_cond(3) = 0;
    B1_cond(3) = 1;

    H = diag([2, 2, 2]);
    f = -2 * Torque_filt_prev;

    A = [diag(A1_cond); diag(A2_cond)];
    b = [B1_cond; B2_cond];

    lb = -Torque_max;
    ub =  Torque_max;

    options = optimoptions('quadprog', 'Display', 'off', 'MaxIterations', 100);
    [u_opt, ~, exitflag] = quadprog(H, f, A, b, [], [], lb, ub, Torque_prev, options);

    if exitflag > 0
        Torque_d    = u_opt;
        Torque_prev = u_opt;
    else
        % QP failed: fallback to saturated nominal torque
        Torque_d    = min(max(Torque_nom, -Torque_max), Torque_max);
        Torque_prev = Torque_d;
        % fprintf('BACKHOCBFQPDOB quadprog failed, exitflag: %d\n', exitflag);
    end

    % CBF-derived torque limits (for logging)
    Torque_lim = [B1_cond ./ A1_cond; B2_cond ./ A2_cond];
    Torque_lim(isinf(Torque_lim) | isnan(Torque_lim)) = 1;
end
