function plot_results(results, p)
% plot_results  Standard result plots for coaxial drone simulation.
%   plot_results(results, p)
%
%   Convention:  actual (solid)  |  desired (b--)  |  max/min (r--, no legend)

t = results.t;

%% 1. 3D Trajectory
figure; hold on; grid on; axis equal;
if p.scenario_id == 4, draw_obstacles(gca); end
plot3(results.X_s(1,:), results.X_s(2,:), results.X_s(3,:), 'LineWidth',2);
plot3(results.X_d_s(1,:), results.X_d_s(2,:), results.X_d_s(3,:), 'b--', 'LineWidth',2);
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
title('3D Trajectory'); view(3);
legend('actual','desired');

%% 2. Position
figure; tiledlayout(3,1); sgtitle('Position');
yLbl = {'X [m]','Y [m]','Z [m]'};
for i = 1:3
    nexttile;
    plot(t, results.X_s(i,:), 'LineWidth',2); hold on;
    plot(t, results.X_d_s(i,:), 'b--', 'LineWidth',2);
    ylabel(yLbl{i}); xlabel('time [s]'); grid on;
    legend('actual','desired');
end

%% 3. Velocity
figure; tiledlayout(3,1); sgtitle('Velocity');
vLbl = {'V_x [m/s]','V_y [m/s]','V_z [m/s]'};
for i = 1:3
    nexttile;
    plot(t, results.Vel_s(i,:), 'LineWidth',2); hold on;
    plot(t, results.Vel_d_s(i,:), 'b--', 'LineWidth',2);
    ylabel(vLbl{i}); xlabel('time [s]'); grid on;
    legend('actual','desired');
end

%% 4. Acceleration
figure; tiledlayout(3,1); sgtitle('Acceleration');
acLbl = {'A_x [m/s^2]','A_y [m/s^2]','A_z [m/s^2]'};
for i = 1:3
    nexttile;
    plot(t, results.Acc_s(i,:), 'LineWidth',2); hold on;
    plot(t, results.Acc_d_s(i,:), 'b--', 'LineWidth',2);
    plot(t, results.Acc_Max_s(i,:), 'r--', 'LineWidth',1.5, 'HandleVisibility','off');
    plot(t, -results.Acc_Max_s(i,:), 'r--', 'LineWidth',1.5, 'HandleVisibility','off');
    m = max([abs(results.Acc_s(i,:)), abs(results.Acc_d_s(i,:)), eps]);
    ylim([-m*1.1, m*1.1]);
    ylabel(acLbl{i}); xlabel('time [s]'); grid on;
    legend('actual','desired');
end

%% 5. Attitude
figure; tiledlayout(3,1); sgtitle('Attitude');
aLbl = {'\phi [deg]','\theta [deg]','\psi [deg]'};
for i = 1:3
    nexttile; idx = i+3;
    act = rad2deg(results.X_s(idx,:));
    des = rad2deg(results.X_d_s(idx,:));
    plot(t, act, 'LineWidth',2); hold on;
    plot(t, des, 'b--', 'LineWidth',2);
    bnd = rad2deg(p.angle_max(i));
    yline(bnd,'r--','LineWidth',1.5,'HandleVisibility','off');
    yline(-bnd,'r--','LineWidth',1.5,'HandleVisibility','off');
    m = max([abs(act), abs(des), eps]);
    ylim([-m*1.1, m*1.1]);
    ylabel(aLbl{i}); xlabel('time [s]'); grid on;
    legend('actual','desired');
end

%% 6. Angular Velocity
figure; tiledlayout(3,1); sgtitle('Angular Velocity');
wLbl = {'p [deg/s]','q [deg/s]','r [deg/s]'};
for i = 1:3
    nexttile; idx = i+9;
    act = rad2deg(results.X_s(idx,:));
    des = rad2deg(results.X_d_s(idx,:));
    plot(t, act, 'LineWidth',2); hold on;
    plot(t, des, 'b--', 'LineWidth',2);
    bnd = rad2deg(p.angle_dot_max(i));
    yline(bnd,'r--','LineWidth',1.5,'HandleVisibility','off');
    yline(-bnd,'r--','LineWidth',1.5,'HandleVisibility','off');
    m = max([abs(act), abs(des), eps]);
    ylim([-m*1.1, m*1.1]);
    ylabel(wLbl{i}); xlabel('time [s]'); grid on;
    legend('actual','desired');
end

%% 7. Torque
figure; tiledlayout(3,1); sgtitle('Torque');
tLbl = {'\tau_\phi [Nm]','\tau_\theta [Nm]','\tau_\psi [Nm]'};
for i = 1:3
    nexttile;
    cmd   = results.torque_d_s(i,:);        % controller command
    rotor = results.torque_rotor_s(i,:);    % rotor-generated torque
    total = results.torque_total_s(i,:);    % rotor + disturbance
    plot(t, cmd, 'b--', 'LineWidth',2); hold on;
    plot(t, rotor, 'LineWidth',1.5);
    plot(t, total, 'k', 'LineWidth',1);
    plot(t, results.Torque_max_s(i,:), 'r--', 'LineWidth',1.5, 'HandleVisibility','off');
    plot(t, -results.Torque_max_s(i,:), 'r--', 'LineWidth',1.5, 'HandleVisibility','off');
    m = max([abs(cmd), abs(total), eps]);
    ylim([-m*1.1, m*1.1]);
    ylabel(tLbl{i}); xlabel('time [s]'); grid on;
    legend('command','rotor','total');
end

%% 8. Rotor Input
figure; tiledlayout(4,1); sgtitle('Rotor Input');
uIdx = [13,14,15,16];
uLbl = {'\alpha [deg]','\beta [deg]','\Omega_1 [deg/s]','\Omega_2 [deg/s]'};
mIdx = [1,2,3,3]; twoSide = [true,true,false,false];
for i = 1:4
    nexttile;
    act = rad2deg(results.X_s(uIdx(i),:));
    des = rad2deg(results.X_d_s(uIdx(i),:));
    plot(t, act, 'LineWidth',2); hold on;
    plot(t, des, 'b--', 'LineWidth',2);
    ym = rad2deg(p.rot_max(mIdx(i)));
    yline(ym,'r--','LineWidth',1.5,'HandleVisibility','off');
    if twoSide(i), yline(-ym,'r--','LineWidth',1.5,'HandleVisibility','off'); end
    m = max([abs(act), abs(des), eps]);
    if twoSide(i)
        ylim([-m*1.1, m*1.1]);
    else
        ylim([0, m*1.1]);
    end
    ylabel(uLbl{i}); xlabel('time [s]'); grid on;
    legend('actual','desired');
end

end
