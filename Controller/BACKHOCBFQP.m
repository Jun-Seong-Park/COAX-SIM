function [Torque_d, Torque_nom, Torque_lim] = BACKHOCBFQP(state, state_1dot, Desired, state_max, p)
% BACKHOCBFQP  Backstepping + 1st-order filter + HOCBF-QP (no DOB)
%   [Torque_d, Torque_nom, Torque_lim] = BACKHOCBFQP(state, state_1dot, Desired, state_max, p)
%
%   Inputs:
%     state      [3x1] : attitude angles [phi; theta; psi]
%     state_1dot [3x1] : attitude rates  [p; q; r]
%     Desired    [3x3] : [Theta_d, Theta_d_dot, Theta_d_ddot]
%     state_max  [3x3] : [angle_max, angle_dot_max, Torque_max]
%     p          struct: params (uses Inertia, dt_ctrl, K1_cbf, K2_cbf, P1_cbf, P2_cbf, P3_cbf, tau_filt)
%
%   Outputs:
%     Torque_d   [3x1] : QP-constrained control torque
%     Torque_nom [3x1] : nominal backstepping torque (before QP)
%     Torque_lim [6x1] : CBF-derived torque limits [B1_cond./A1; B2_cond./A2]

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

    % Nominal torque (unconstrained backstepping)
    Torque_nom = -p.Inertia * (Z1 - Theta_2d_dd + K2 * Z2);

    % 1st-order low-pass filter
    Torque_filt_prev = Torque_filt_prev + p.dt_ctrl * (Torque_nom - Torque_filt_prev) / p.tau_filt;

    % --- HOCBF constraints ---
    % B1: attitude barrier  (relative degree 2)
    %   psi1     = B1_dot + P1*B1
    %   psi1_dot = B1_ddot + P1*B1_dot
    %   psi2     = psi1_dot + P2*psi1 >= 0
    %   => diag(2*Theta)*tau <= J*(-2*Theta_dot.^2 - 2*(P1+P2)*Theta.*Theta_dot + P1*P2*B1)
    B1 = Theta_max.^2 - Theta.^2;
    B2 = Theta_dot_max.^2 - Theta_dot.^2;

    A1_cond = 2 * Theta;
    A2_cond = 2 * Theta_dot;
    B1_cond = p.Inertia * (-2 * Theta_dot.^2 + (P1 + P2) * (-2 * Theta .* Theta_dot) + P1 * P2 * B1);
    B2_cond = p.Inertia * P3 * B2;

    % Exclude psi from B1 (yaw angle barrier not physical with large theta_max)
    A1_cond(3) = 0;
    B1_cond(3) = 1;

    % QP: min ||tau - tau_filt||^2  s.t.  A*tau <= b,  lb <= tau <= ub
    H = diag([2, 2, 2]);
    f = -2 * Torque_filt_prev;

    A = [diag(A1_cond); diag(A2_cond)];
    b = [B1_cond; B2_cond];

    lb = -Torque_max;
    ub =  Torque_max;

    options = optimoptions('quadprog', 'Display', 'off', 'MaxIterations', 100);
    [u_opt, ~, exitflag] = quadprog(H, f, A, b, [], [], lb, ub, Torque_prev, options);

    Torque_d = zeros(3,1);
    if exitflag > 0
        Torque_d     = u_opt;
        Torque_prev  = u_opt;
    else
        % fprintf('BACKHOCBFQP quadprog failed, exitflag: %d\n', exitflag);
    end

    % CBF-derived torque limits (for logging)
    Torque_lim = [B1_cond ./ A1_cond; B2_cond ./ A2_cond];
    Torque_lim(isinf(Torque_lim) | isnan(Torque_lim)) = 1;
end
