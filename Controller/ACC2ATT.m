function att_d = ACC2ATT(u, a_T, psi_d)
% ACC2ATT  Desired acceleration -> desired attitude angles
%   No external parameters required.

    phi_d   = asin(1/a_T * (u(1)*sin(psi_d) - u(2)*cos(psi_d)));
    theta_d = asin(1/a_T * (u(1)*cos(psi_d) + u(2)*sin(psi_d)) / cos(phi_d));
    att_d   = [phi_d; theta_d; psi_d];
end
