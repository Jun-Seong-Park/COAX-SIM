function X_d = Scenario2(t)
%% Scenario 2 — Circle
% 목표: 5m 고도, R=5m 원형 궤적, yaw 고정
% 반환: [x; y; z; yaw_d]

    z_d    = -5;    % 비행 고도 (NED)
    R      = 5;     % 반지름 (m)
    omega  = 0.1;   % 각속도 (rad/s) → 주기 ≈ 62.8s
    t_rise = 10;    % 원 시작점까지 이동 시간 (s)

    if t < t_rise
        % Phase 1: 원 시작점 [R, 0]으로 이동
        ratio = t / t_rise;
        X_d = [R * ratio; 0; z_d; 0];
    else
        % Phase 2: 원형 궤적
        t_fly = t - t_rise;
        X_d = [R * cos(omega * t_fly); ...
               R * sin(omega * t_fly); ...
               z_d; 0];
    end
end
