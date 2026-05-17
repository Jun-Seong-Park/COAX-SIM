function X_d = Scenario4(t)
%% Scenario 4 — RRT Path
% rrt_traj_ref.csv (t, x, y, z) 고정 로드
% CSV z열: 위가 양수 → NED 변환 (부호 반전)
% 반환: [x; y; z; yaw_d]

    persistent t_data x_data y_data z_data

    if isempty(t_data)
        % 현재 폴더 기준으로 파일 탐색
        csv_file = 'rrt_traj_ref.csv';
        if ~isfile(csv_file)
            % Trajectory 폴더 내 탐색
            csv_file = fullfile('Trajectory', 'rrt_traj_ref.csv');
        end
        if ~isfile(csv_file)
            error('Scenario8: rrt_traj_ref.csv 파일을 찾을 수 없습니다.');
        end

        data   = readmatrix(csv_file);
        t_data = data(:,1);
        x_data = data(:,2);
        y_data = data(:,3);
        z_data = -data(:,4);   % 위가 양수 → NED (아래 양수)
    end

    if t <= t_data(end)
        x_d = interp1(t_data, x_data, t, 'linear', 'extrap');
        y_d = interp1(t_data, y_data, t, 'linear', 'extrap');
        z_d = interp1(t_data, z_data, t, 'linear', 'extrap');

        % Heading = trajectory direction (numerical derivative)
        dt_look = 0.5;      % look-ahead [s]
        t_next = min(t + dt_look, t_data(end));
        x_next = interp1(t_data, x_data, t_next, 'linear', 'extrap');
        y_next = interp1(t_data, y_data, t_next, 'linear', 'extrap');
        dx = x_next - x_d;
        dy = y_next - y_d;
        if dx^2 + dy^2 > 1e-6
            yaw_d = atan2(dy, dx);
        else
            yaw_d = 0;      % stationary → keep heading
        end
    else
        x_d = x_data(end);
        y_d = y_data(end);
        z_d = z_data(end);
        yaw_d = 0;
    end

    X_d = [x_d; y_d; z_d; yaw_d];
end
