function rot_d = compute_rotor_speed_CoaX(Thrust_d, torque_d, rot_max, p)
% compute_rotor_speed_CoaX  Desired thrust/torque -> rotor speeds & tilt angles
%   Uses params struct p (fields: k, gamma, r_cd)

    torque_phi_d   = torque_d(1);
    torque_theta_d = torque_d(2);
    torque_psi_d   = torque_d(3);

    [alpha_max, beta_max, Omega_max] = deal(rot_max(1), rot_max(2), rot_max(3));

    k_1 = p.k(1);       k_2 = p.k(2);
    gamma_1 = p.gamma(1);  gamma_2 = p.gamma(2);

    Omega_up_d = sqrt((-gamma_2 * Thrust_d + k_2 * torque_psi_d) / ...
                      (k_1 * gamma_2 + k_2 * gamma_1));
    Omega_up_d = min(max(Omega_up_d, 0), Omega_max);

    Omega_dw_d = sqrt((-gamma_1 * Thrust_d - k_1 * torque_psi_d) / ...
                      (k_1 * gamma_2 + k_2 * gamma_1));
    Omega_dw_d = min(max(Omega_dw_d, 0), Omega_max);

    alpha_d = torque_phi_d   / k_2 / Omega_dw_d^2 / p.r_cd;
    alpha_d = min(max(alpha_d, -alpha_max), alpha_max);

    beta_d  = torque_theta_d / k_2 / Omega_dw_d^2 / p.r_cd;
    beta_d  = min(max(beta_d, -beta_max), beta_max);

    rot_d = [alpha_d; beta_d; Omega_up_d; Omega_dw_d];
end
