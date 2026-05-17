function results = run_sim(params)
% run_sim  Run coaxial drone simulation with given parameters
%   results = run_sim(p)
%
%   Input:
%     p       struct from init_params() (fields may be overridden by GUI)
%
%   Output:
%     results struct containing all simulation data for plotting/analysis
%
%   Usage (standalone):
%     p = init_params();
%     results = run_sim(p);
%     plot_results(results.t, results.X_s, results.X_d_s, ...);
%
%   Usage (from GUI):
%     p = init_params();
%     p.ctrl_mode = 3;        % override from GUI
%     p.K1_bsc = [10;10;10];  % override from GUI
%     results = run_sim(p);

    % Clear persistent variables in controllers
    clear BACKHOCBFQP BACKHOCBFQPDOB

    % Absolute path to avoid loading old (global-based) functions from parent dir
    sim_dir = fileparts(mfilename('fullpath'));
    addpath(fullfile(sim_dir,'Controller'), ...
            fullfile(sim_dir,'Dynamics'), ...
            fullfile(sim_dir,'Trajectory'), ...
            fullfile(sim_dir,'Scenario'), ...
            fullfile(sim_dir,'Plot'), ...
            fullfile(sim_dir,'Animation'))

    %% Time vectors
    t_sim  = params.t0 : params.dt_sim : params.tf;
    n_sim  = length(t_sim);
    ctrl_steps  = round(params.dt_ctrl  / params.dt_sim);
    rot_steps   = round(params.dt_rot   / params.dt_sim);
    servo_steps = round(params.dt_servo / params.dt_sim);
    save_steps  = round(params.dt_save  / params.dt_sim);

    %% State initialisation
    X      = zeros(16,1);
    X_d    = zeros(16,1);

    % Set initial position: xy from scenario, z = 0 (ground)
    Xd_init      = scenario_call(0, params.scenario_id);
    X(1:2)       = Xd_init(1:2);        % start at scenario's xy
    X(3)         = 0;                   % start at ground (NED z=0)

    %% Controller state
    torque_d   = zeros(3,1);
    filt_state = zeros(6,1);
    z_dob      = zeros(3,1);
    z_dob_pos  = zeros(3,1);        % LDOB_pos internal state
    Desired    = zeros(3,3);
    d_hat      = zeros(3,1);
    d_hat_f    = zeros(3,1);        % force disturbance estimate (NED)
    Acc_d_prev = zeros(3,1);        % previous Acc_d for LDOB_pos

    %% Observer initialisation
    % obs_mode 1: true state + true disturbance (perfect info)
    % obs_mode 2: noisy measurement + LDOB (realistic)
    Disturbance_prev = zeros(6,1);

    ctrl_counter = 0;  ctrl_time_total = 0;  ctrl_time_max = 0;
    save_counter = 0;

    %% Torque max (initial, updated in loop)
    Torque_max = [params.r_cd * params.k(1) * params.Omega_hover^2 * sin(params.tilt_max(1));
                  params.r_cd * params.k(2) * params.Omega_hover^2 * sin(params.tilt_max(2));
                  0.2];
    state_max  = [params.angle_max, params.angle_dot_max, Torque_max];

    %% Pre-allocate save arrays
    n_save = ceil(n_sim / save_steps);
    X_s = zeros(16, n_save);   X_d_s = zeros(16, n_save);
    torque_d_s = zeros(3, n_save);
    Acc_d_s = zeros(3, n_save); Acc_s = zeros(3, n_save);
    Force_s = zeros(3, n_save); Moment_s = zeros(3, n_save);
    torque_rotor_s = zeros(3, n_save);  % rotor-only torque
    torque_total_s = zeros(3, n_save);  % rotor + disturbance (total applied)
    d_hat_s = zeros(3, n_save); Disturbance_s = zeros(6, n_save);
    B1_s = zeros(3, n_save);   B2_s = zeros(3, n_save);
    Torque_max_s = zeros(3, n_save);
    Acc_Max_s = zeros(3, n_save);
    Thrust_d_s = zeros(1, n_save);
    Vel_d_s = zeros(3, n_save);  Vel_s = zeros(3, n_save);

    %% MPC initialisation (only when needed)
    if params.ctrl_mode == 4
        [x_mpc, mpcobj] = mpcset(state_max, params);
    end

    %% Main loop
    for i = 1:n_sim-1
        time = i * params.dt_sim;
        if rem(time,1) < 1e-6, fprintf('time: %gs\n', time); end

        % Unpack
        [phi,theta,psi] = deal(X(4), X(5), X(6));
        [alpha,beta,Omega_up,Omega_dw] = deal(X(13), X(14), X(15), X(16));
        eul = [psi, theta, phi];
        R_bn = eul2rotm(eul);              % Rotation matrix: body → NED 
        GravityForce = R_bn' * params.Mass * [0;0;params.g];  % gravity in body frame [N]

        % Desired trajectory
        Xd4D       = scenario_call(time, params.scenario_id);
        X_d(1:3,:) = Xd4D(1:3);
        X_d(6,:) = Xd4D(4);

        %% ---- Control loop ----
        if mod(i-1, ctrl_steps) == 0
            tic;

            % Update torque limits
            state_max(:,3) = TorqueMax(Omega_up, Omega_dw, params.rot_max, params);
            Torque_max = state_max(:,3);

            %% ---- Observer ----
            pos_est   = X(1:3);
            vel_est   = R_bn * X(7:9);
            att_est   = X(4:6);
            omega_est = X(10:12);

            switch params.obs_mode
                case 1  % True state + true disturbance (perfect info)
                    d_hat   = Disturbance_prev(4:6);
                    d_hat_f = Disturbance_prev(1:3);

                case 2  % Noisy measurement + LDOB
                    pos_est   = pos_est   + params.sigma_pos  .* randn(3,1);
                    vel_est   = vel_est   + params.sigma_vel  .* randn(3,1);
                    att_est   = att_est   + params.sigma_att  .* randn(3,1);
                    omega_est = omega_est + params.sigma_gyro  * randn(3,1);
                    [d_hat,   z_dob]     = LDOB(omega_est, torque_d, z_dob, params);
                    [d_hat_f, z_dob_pos] = LDOB_pos(vel_est, Acc_d_prev, z_dob_pos, params);
            end

            %% ---- Guidance ----
            [Acc_d, Acc_Max, THR_d, Vel_d] = POS2THR(pos_est, vel_est, X_d(1:3), params.angle_max, d_hat_f, params);
            Acc_d_prev = Acc_d;
            a_T   = THR_d / params.Mass;
            att_d = ACC2ATT(Acc_d, a_T, psi);          % phi_d, theta_d from current yaw
            X_d(4:5,:) = att_d(1:2);                    % keep X_d(6) = scenario yaw_d

            % Command filter
            [Desired, filt_state] = CmdFilter(X_d(4:6), filt_state, params.dt_ctrl, params.angle_max, params.angle_dot_max, Torque_max, params);
            X_d(10:12) = Desired(:,2);

            %% ---- Attitude controller ----
            switch params.ctrl_mode
                case 1
                    torque_d = BACKDOB(att_est, omega_est, Desired, Torque_max, d_hat, params.K1_bsc, params.K2_bsc, params);
                case 2
                    [torque_d, ~, ~] = BACKHOCBFQP(att_est, omega_est, Desired, state_max, params);
                case 3
                    [torque_d, ~, ~] = BACKHOCBFQPDOB(att_est, omega_est, Desired, state_max, d_hat, params);
                case 4
                    torque_d = MPCctrl(att_est, omega_est, Desired, mpcobj, x_mpc);
            end

            % Rotor commands
            X_d(13:16,:) = compute_rotor_speed_CoaX(THR_d, torque_d, params.rot_max, params);
            if params.rotor_dyn_mode == 1
                X(13:16,:) = X_d(13:16,:);          % instant, no lag
            end

            ctrl_time = toc;
            ctrl_time_total = ctrl_time_total + ctrl_time;
            ctrl_counter    = ctrl_counter + 1;
            ctrl_time_max   = max(ctrl_time_max, ctrl_time);
        end

        %% ---- Rotor dynamics (dt_rot period) ----
        if params.rotor_dyn_mode >= 2 && mod(i-1, rot_steps) == 0
            switch params.rotor_dyn_mode
                case 2  % First-order lag
                    X(15:16,:) = RotorActDyn(X(15:16,:), X_d(15:16,:), params.tau_rot, params.dt_rot);
                case 3  % Schafroth model
                    X(15:16,:) = RotorDynSchafroth(X(15:16,:), X_d(15:16,:), params.dt_rot, params);
            end
        end

        %% ---- Servo dynamics (dt_servo period) ----
        if params.rotor_dyn_mode >= 2 && mod(i-1, servo_steps) == 0
            X(13:14,:) = ServoActDyn(X(13:14,:), X_d(13:14,:), params.tau_sw, params.dt_servo);
        end

        %% ---- Dynamics (use current actuator states) ----
        [Force_body, Moment_body] = CoaX_Dyn(X(15), X(16), X(13), X(14), params);
        Moment_rotor = Moment_body;             % rotor-only torque (before disturbance)
        Disturbance     = DisGen(params.Dis_max, time, params.dis_mode);
        Force_body  = Force_body  + Disturbance(1:3) + GravityForce;
        Moment_body = Moment_body + Disturbance(4:6);
        [X(1:12,:), ~] = sixdof(X(1:12,:), Force_body, Moment_body, params.dt_sim, params);
        Disturbance_prev = Disturbance;

        %% ---- Save ----
        if mod(i, save_steps) == 0
            save_counter = save_counter + 1;
            sc = save_counter;
            X_s(:,sc)  = X;  X_s(3,sc) = -X(3);
            X_d_s(:,sc) = X_d; X_d_s(3,sc) = -X_d(3);
            torque_d_s(:,sc) = torque_d;
            Thrust_d_s(sc)   = THR_d;
            Acc_d_s(:,sc) = Acc_d; Acc_d_s(3,sc) = -Acc_d(3);
            Acc = R_bn * (Force_body / params.Mass);
            if params.obs_mode == 3
                Acc = Acc + params.sigma_acc .* randn(3,1);
            end
            Acc_s(:,sc) = Acc; Acc_s(3,sc) = -Acc(3);
            Force_s(:,sc) = Force_body;  Moment_s(:,sc) = Moment_body;
            torque_rotor_s(:,sc) = Moment_rotor;
            torque_total_s(:,sc) = Moment_body(1:3);  % rotor + disturbance
            B1_s(:,sc) = state_max(:,1).^2 - X(4:6).^2;
            B2_s(:,sc) = state_max(:,2).^2 - X(10:12).^2;
            Torque_max_s(:,sc) = Torque_max;
            Acc_Max_s(:,sc)    = Acc_Max;
            Disturbance_s(:,sc)= Disturbance;
            d_hat_s(:,sc)      = d_hat;
            Vel_d_s(:,sc) = Vel_d; Vel_d_s(3,sc) = -Vel_d(3);
            Vel_NED_save  = R_bn * X(7:9);
            Vel_s(:,sc)   = Vel_NED_save; Vel_s(3,sc) = -Vel_NED_save(3);
        end
    end

    %% Build results struct
    sc = save_counter;
    results.t           = params.t0 : params.dt_save : (sc-1)*params.dt_save;
    results.X_s         = X_s(:,1:sc);
    results.X_d_s       = X_d_s(:,1:sc);
    results.torque_d_s     = torque_d_s(:,1:sc);
    results.torque_rotor_s = torque_rotor_s(:,1:sc);
    results.torque_total_s = torque_total_s(:,1:sc);
    results.Moment_s       = Moment_s(:,1:sc);
    results.Acc_s       = Acc_s(:,1:sc);
    results.Acc_d_s     = Acc_d_s(:,1:sc);
    results.Acc_Max_s   = Acc_Max_s(:,1:sc);
    results.Force_s     = Force_s(:,1:sc);
    results.d_hat_s     = d_hat_s(:,1:sc);
    results.Disturbance_s = Disturbance_s(:,1:sc);
    results.B1_s        = B1_s(:,1:sc);
    results.B2_s        = B2_s(:,1:sc);
    results.Torque_max_s = Torque_max_s(:,1:sc);
    results.Thrust_d_s  = Thrust_d_s(1:sc);
    results.Vel_s       = Vel_s(:,1:sc);
    results.Vel_d_s     = Vel_d_s(:,1:sc);

    results.ctrl_time_mean = ctrl_time_total / ctrl_counter * 1000;
    results.ctrl_time_max  = ctrl_time_max * 1000;
end
