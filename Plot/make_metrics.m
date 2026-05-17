function metrics = make_metrics(results, p)
% make_metrics  Compute and display simulation performance metrics
%   metrics = make_metrics(results, p)
%
%   Steady-state detection: ||err|| < 0.5m sustained for 1s
%   Reports both full-trajectory and steady-state RMSE.

    t  = results.t;
    dt = p.dt_save;
    n  = length(t);

    %% Position error
    err      = results.X_s(1:3,:) - results.X_d_s(1:3,:);
    err_norm = vecnorm(err, 2, 1);

    %% Full-trajectory RMSE
    metrics.rmse_pos   = sqrt(mean(err.^2, 2));
    metrics.rmse_total = sqrt(mean(sum(err.^2, 1)));
    metrics.max_err    = max(abs(err), [], 2);

    %% Steady-state detection: ||err|| < eps_ss for T_ss seconds
    eps_ss = 0.5;           % [m] steady-state threshold
    T_ss   = 1.0;           % [s] hold duration
    N_hold = round(T_ss / dt);

    is_settled = err_norm < eps_ss;
    ss_mask = false(1, n);
    for i = N_hold:n
        if all(is_settled(i-N_hold+1:i))
            ss_mask(i) = true;
        end
    end

    %% Steady-state RMSE
    if any(ss_mask)
        err_ss = err(:, ss_mask);
        metrics.rmse_ss_pos   = sqrt(mean(err_ss.^2, 2));
        metrics.rmse_ss_total = sqrt(mean(sum(err_ss.^2, 1)));
        metrics.ss_ratio      = sum(ss_mask) / n * 100;    % % of time in SS
    else
        metrics.rmse_ss_pos   = NaN(3,1);
        metrics.rmse_ss_total = NaN;
        metrics.ss_ratio      = 0;
    end

    %% Timing
    metrics.sim_time  = t(end);
    metrics.ctrl_mean = results.ctrl_time_mean;
    metrics.ctrl_max  = results.ctrl_time_max;

    %% First settling time (from t=0)
    e0 = err_norm(1);
    if e0 > 0.01
        idx_settled = find(ss_mask, 1, 'first');
        if ~isempty(idx_settled)
            metrics.settling = t(idx_settled);
        else
            metrics.settling = NaN;
        end
    else
        metrics.settling = 0;
    end

    %% Attitude constraint violation
    angle_max_rp = p.angle_max(1);
    phi_viol   = max(0, abs(results.X_s(4,:)) - angle_max_rp);
    theta_viol = max(0, abs(results.X_s(5,:)) - angle_max_rp);
    metrics.att_violation_sum = [sum(phi_viol); sum(theta_viol)] * dt;

    %% Print
    fprintf('\n===== Simulation Metrics =====\n');
    fprintf('Sim time:       %.1f s\n', metrics.sim_time);
    fprintf('Ctrl loop:      %.3f ms (mean), %.3f ms (max)\n', metrics.ctrl_mean, metrics.ctrl_max);
    fprintf('Settling (%.1fm): ', eps_ss);
    if isnan(metrics.settling), fprintf('N/A\n'); else, fprintf('%.2f s\n', metrics.settling); end
    fprintf('\n--- Full Trajectory ---\n');
    fprintf('RMSE pos:       X=%.4f  Y=%.4f  Z=%.4f  Total=%.4f [m]\n', ...
        metrics.rmse_pos(1), metrics.rmse_pos(2), metrics.rmse_pos(3), metrics.rmse_total);
    fprintf('Max error:      X=%.4f  Y=%.4f  Z=%.4f [m]\n', ...
        metrics.max_err(1), metrics.max_err(2), metrics.max_err(3));
    fprintf('\n--- Steady State (||err||<%.1fm, %.0fs hold) ---\n', eps_ss, T_ss);
    if any(ss_mask)
        fprintf('SS ratio:       %.1f%% of sim time\n', metrics.ss_ratio);
        fprintf('RMSE pos (SS):  X=%.4f  Y=%.4f  Z=%.4f  Total=%.4f [m]\n', ...
            metrics.rmse_ss_pos(1), metrics.rmse_ss_pos(2), metrics.rmse_ss_pos(3), metrics.rmse_ss_total);
    else
        fprintf('No steady-state detected.\n');
    end
    fprintf('\nAtt violation:  phi=%.6f  theta=%.6f [rad*s]\n', ...
        metrics.att_violation_sum(1), metrics.att_violation_sum(2));
    fprintf('==============================\n\n');

    %% Plot
    figure; tiledlayout(2,1);

    % Error per axis
    nexttile;
    plot(t, err(1,:), 'r', t, err(2,:), 'g', t, err(3,:), 'b', 'LineWidth', 1.5);
    ylabel('Error [m]'); xlabel('time [s]'); grid on;
    legend('e_x', 'e_y', 'e_z');
    title(sprintf('Position Error (RMSE = %.4f m)', metrics.rmse_total));

    % Error norm + steady-state highlight
    nexttile;
    plot(t, err_norm, 'k', 'LineWidth', 1.5); hold on;
    yline(eps_ss, 'r--', 'LineWidth', 1, 'HandleVisibility', 'off');
    if any(ss_mask)
        % Shade steady-state regions
        ss_t = t; ss_t(~ss_mask) = NaN;
        ss_e = err_norm; ss_e(~ss_mask) = NaN;
        plot(ss_t, ss_e, 'g', 'LineWidth', 3);
        if ~isnan(metrics.rmse_ss_total)
            legend('||error||', sprintf('SS (RMSE=%.4fm, %.0f%%)', metrics.rmse_ss_total, metrics.ss_ratio));
        end
    else
        legend('||error||');
    end
    ylabel('||Error|| [m]'); xlabel('time [s]'); grid on;
    title(sprintf('Error Norm (SS threshold = %.1f m)', eps_ss));
end
