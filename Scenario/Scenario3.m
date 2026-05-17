function X_d = Scenario3(t)
%% Scenario 3 — Circle + Heading
% 목표: R=5m 원형 궤적 + 진행 방향(tangent)으로 yaw 추종
% 반환: [x; y; z; yaw_d]

    z_d    = -5;
    R      = 5;
    omega  = 0.1;   % rad/s
    t_rise = 10;    % 원 시작점까지 이동 시간
    t_wait = 5;     % 시작점 도달 후 대기 시간
    t_go   = t_rise + t_wait;   % 원 궤적 시작 시각

    if t < t_rise
        ratio = t / t_rise;
        X_d = [R * ratio; 0; z_d; 0];
    elseif t < t_go
        X_d = [R; 0; z_d; 0];              % 시작점 대기
    else
        t_fly  = t - t_go;
        angle  = omega * t_fly;

        % 원 궤적
        x_d = R * cos(angle);
        y_d = R * sin(angle);

        % Heading = 진행 방향 tangent
        % vx = -R*omega*sin(angle), vy = R*omega*cos(angle)
        yaw_d = atan2(cos(angle), -sin(angle));

        X_d = [x_d; y_d; z_d; yaw_d];
    end
end
