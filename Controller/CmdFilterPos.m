function [Desired, filt] = CmdFilterPos(pos_d_raw, filt, dt, wn, vel_max)
% CmdFilterPos  Second-order command filter for POSITION reference
%   raw 목표위치 하나로부터 [pos_f, vel_f, acc_f] (위치 desired의 0/1/2차 도함수)를
%   생성한다. 각도용 CmdFilter 의 위치 버전.
%
%   각도용 CmdFilter 와의 차이:
%     1) wn 을 torque/inertia 물리로 계산하지 않고 위치 대역폭으로 직접 지정(인자)
%     2) 위치 reference 자체는 제한하지 않음 (pos 포화 제거) — 속도만 saturate
%
%   Inputs:
%     pos_d_raw [3x1] : raw desired position (NED)
%     filt      [6x1] : filter state [pos_f(1:3); vel_f(4:6)]
%     dt        scalar: filter update period [s]
%     wn        scalar or [3x1] : command-filter bandwidth [rad/s]
%     vel_max   [3x1] : desired velocity limit [m/s]
%   Outputs:
%     Desired   [3x3] : [pos_f, vel_f, acc_f] = [pos_d, pd_dot, pd_ddot]
%     filt      [6x1] : updated filter state

    zeta = 1.0;                 % critically damped (no overshoot)

    pos_f = filt(1:3);
    vel_f = filt(4:6);

    % 2nd-order filter: 위치 오차로부터 가속도
    acc_f = -2*zeta*wn .* vel_f - wn.^2 .* (pos_f - pos_d_raw);

    % integrate (forward Euler)
    pos_f = pos_f + dt * vel_f;
    vel_f = vel_f + dt * acc_f;

    % saturate velocity only (position reference is free)
    vel_f = min(max(vel_f, -vel_max), vel_max);

    % recompute acceleration after saturation
    acc_f = -2*zeta*wn .* vel_f - wn.^2 .* (pos_f - pos_d_raw);

    filt(1:3) = pos_f;
    filt(4:6) = vel_f;

    Desired = [pos_f, vel_f, acc_f];
end
