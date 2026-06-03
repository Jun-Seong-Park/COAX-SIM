%% test.m - 1축 위치 Backstepping 제어기 + DOB(외란관측기)
%  plant : pdot = v ,  vdot = u + d   (u = 가속도 입력 a)
%  목표  : pos -> p_d  (위치 제어만, 단일 1축)
clear; clc; close all;

%% ---------------- 시뮬레이션 설정 ----------------
dt = 0.001;           % time step
T  = 10;              % total time [s]
N  = round(T/dt);
t  = (0:N-1)*dt;

%% ---------------- 제어 게인 (Backstepping) -------
K1 = 10;               % outer loop (위치 오차) 게인
K2 = 3;               % inner loop (속도 오차) 게인

%% ---------------- DOB 게인 ------
L = 0.9;             % Q-filter Gain

%% ---------------- 목표 위치 (setpoint) -----------
p_d     = 3;          % desired position (상수; 시간함수로 바꾸면 수치미분 자동작동)
p_d_dot   = 0;
v_d     = 0;
v_d_dot  = 0;

%% ---------------- 초기 상태 ----------------------
pos   = 0;
vel   = 0;
acc    = 0;
vel_prev = 0;         % 이전 속도 (차분미분용)
d_hat    = 0;         % 외란 추정 (EMA 상태)
u_prev   = 0;         % 이전 제어입력
p_d_prev    = p_d;    % 이전 목표위치 (desired 차분미분용)
p_d_dot_prev = 0;      % 이전 목표속도 (desired 2차 차분용)

%% ---------------- 제한 ----------------------
v_max = 3;
u_max = 5;

%% ---------------- 로그 버퍼 ----------------------
P  = zeros(1,N);  V  = zeros(1,N);  A  = zeros(1,N);
D  = zeros(1,N);  Dh = zeros(1,N);
Alp = zeros(1,N); U  = zeros(1,N);
Pd = zeros(1,N);  Pd_dot = zeros(1,N);  Pd_ddot = zeros(1,N);  % desired 신호 (p_d, ṗ_d, p̈_d)
E  = zeros(1,N);                                               % 추종오차 z1 = pos - p_d
A_ideal = zeros(1,N);                                         % 외란보상 전 이상 가속도

%% ================= 메인 루프 =====================
for k = 1:N
    tk = t(k);
    p_d = 3 + 1*sin(tk);
    % --- 실제 외란 d = sin(t) ---
    d = 2*sin(tk);

    % DOB
    v_dot = (vel - vel_prev)/dt;
    % d_hat     = (1-L)*d_hat + L*v_dot - u_prev;
    d_hat = (1-L)*d_hat + L*(v_dot - u_prev);

    % --- desired 수치미분 ---
    p_d_dot  = (p_d - p_d_prev)/dt;    % 1차 차분
    p_d_ddot = (p_d_dot - p_d_dot_prev)/dt;  % 2차 차분

    % --- Backstepping ---
    z1        = pos - p_d;                
    v_d       = p_d_dot - K1*z1;           
    v_d = min(max(v_d, -v_max), v_max);
    z2        = vel - v_d;                 
    v_d_dot = p_d_ddot - K1*(vel - p_d_dot);
    a_ideal = v_d_dot - K2*z2 - z1;       % 외란보상 전 이상 가속도 (추종 대상)
    u       = a_ideal - d_hat;            % DOB 보상 포함 명령 (= a_des)
    u = min(max(u, -u_max), u_max);

    acc = u + d;                 
    
    % --- 로그 ---  Alp=v_d, U=a_des, A=실제가속도
    P(k)=pos; V(k)=vel; Alp(k)=v_d; U(k)=u; A(k)=acc;
    D(k)=d;   Dh(k)=d_hat;
    Pd(k)=p_d; Pd_dot(k)=p_d_dot; Pd_ddot(k)=p_d_ddot;  % desired 신호
    E(k)=z1;  A_ideal(k)=a_ideal;                        % 추종오차, 이상 가속도

    % --- 플랜트 적분 (forward Euler) ---
    pos      = pos + vel*dt;
    vel_prev = vel;          % v_k 저장 (다음 스텝 차분미분용)
    vel      = vel + acc*dt;
    u_prev   = u;            % 이전 제어입력 저장
    p_d_prev    = p_d;   % 목표위치 저장 (다음 스텝 차분용)
    p_d_dot_prev = p_d_dot;    % 목표속도 저장
end

%% ================= 결과 plot (4x1) ===============
figure('Name','BSC + DOB (1-axis)');

% (1) position : p vs p_d(궤적)
subplot(4,1,1); hold on; grid on;
plot(t,P, 'b','LineWidth',1.3);
plot(t,Pd,'k--','LineWidth',1.3);
ylabel('p [m]'); title('Position (solid: p,  dashed: p_d)');
legend({'p','p_d'},'Location','best');

% (2) velocity : v, v_d(alpha=가상명령), pd_dot(순수 desired)
subplot(4,1,2); hold on; grid on;
plot(t,V,      'b','LineWidth',1.3);
plot(t,Alp,    'r--','LineWidth',1.3);
plot(t,Pd_dot, 'g:','LineWidth',1.0);
ylabel('v [m/s]'); title('Velocity (v,  v\_d=\alpha,  pd\_dot)');
legend({'v','v_d=\alpha','pd\_dot'},'Location','best');

% (3) acceleration : a_actual(u+d) vs a_cmd(u) vs a_ideal(target)
subplot(4,1,3); hold on; grid on;
mi = 1:500:N;   % marker 위치 (중간중간 찍기)
plot(t,A,      'b','LineWidth',1.3,'Marker','pentagram','MarkerIndices',mi,'MarkerSize',9);
plot(t,U,      'r--','LineWidth',1.3,'Marker','square','MarkerIndices',mi,'MarkerSize',7);
plot(t,A_ideal,'g:','LineWidth',1.4,'Marker','diamond','MarkerIndices',mi,'MarkerSize',7);
ylim([-10 10]);
ylabel('a [m/s^2]'); title('Acceleration (a\_actual=u+d,  a\_cmd=u,  a\_ideal=target)');
legend({'a_{actual}=u+d','a_{cmd}=u','a_{ideal} (target)'},'Location','best');

% (4) disturbance : d vs d_hat
subplot(4,1,4); hold on; grid on;
plot(t,D, 'b','LineWidth',1.3);
plot(t,Dh,'r--','LineWidth',1.3);
ylabel('d'); xlabel('time [s]');
title('Disturbance (solid: d,  dashed: d\_hat)');
legend({'d','d\_hat'},'Location','best');

saveas(gcf, 'test_result.png');
fprintf('final pos error = %.4f\n', (pos-p_d));
fprintf('RMS tracking error = %.4f\n', sqrt(mean(E.^2)));
fprintf('max |tracking error| = %.4f\n', max(abs(E)));
fprintf('RMS(a_actual - a_ideal) = %.4f  (= DOB 잔차 d-d_hat)\n', sqrt(mean((A-A_ideal).^2)));
fprintf('saved: test_result.png\n');
