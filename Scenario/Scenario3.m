function X_d = Scenario3(t)
%% Scenario 3 — Circle + Heading
% Goal: Circle trajectory + heading goal
% output: [x; y; z; yaw_d] [meter, rad]

    z_d    = -1; % Desired height [meter] must be negative, NED
    radius      = 2;  % Desired radius [meter]
    
    t_rise = 5;    % 원 시작점까지 이동 시간
    t_wait = 5;     % 시작점 도달 후 대기 시간
    t_go   = t_rise + t_wait;   % 원 궤적 시작 시각
    t_end = 60;
    
    t_circle = t_end - t_go;                 % 한 바퀴에 주어진 시간 [s]
    virtual_angvel_max = 2*pi / t_circle;    % 2π를 t_circle 안에 완주하도록 역산

    if t < t_rise
        X_d = [0; 0; z_d; 0];
    elseif t < t_go
        X_d = [radius; 0; z_d; 0];              
    else
        t_fly  = t - t_go;
        angle  = virtual_angvel_max * t_fly;
        angle  = min(angle, 2*pi);         % stop after exactly one revolution
        x_d = radius * cos(angle);
        y_d = radius * sin(angle);
        yaw_d = atan2(cos(angle), -sin(angle));
        X_d = [x_d; y_d; z_d; yaw_d];
    end
end
