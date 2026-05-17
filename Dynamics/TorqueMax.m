% Maximum Torque Calculation (refactored: params struct p instead of globals)

function Torque_max = TorqueMax(Omega_up, Omega_dw, rot_max, p)

alpha_max = rot_max(1);
beta_max = rot_max(2);

Torque_max = zeros(3,1);
Torque_max(1) = p.r_cd * p.k(2) * Omega_dw^2 * sin(alpha_max);
Torque_max(2) = p.r_cd * p.k(2) * Omega_dw^2 * sin(beta_max);
Torque_max(3) = 0.2;

end
