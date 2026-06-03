%% verify_ldob.m  -  LDOB 코드 <-> Q-filter 전달함수 1:1 수치검증
%  방법A: test.m 과 동일한 LDOB 재귀(상태공간)
%  방법B: Q-filter DOB 전달함수  d_hat = Q(s)*(Pn^-1 * v - u),  Q=L/(s+L), Pn^-1 = M*s
%  두 결과가 일치하면 코드가 Q-filter DOB 수식의 구현임이 실증된다.
clear; clc; close all;

dt = 0.001; T = 10; N = round(T/dt); t = (0:N-1)*dt;
L = 0.6;  M = 1;

% --- 임의 입력 신호 (LDOB 와 무관하게 외부 주입) ---
v    = sin(2*t) + 0.5*cos(5*t);        % 속도 신호 v(t)
vdot = 2*cos(2*t) - 2.5*sin(5*t);      % v 의 해석 도함수 (방법B 기준)
u    = 0.3*sin(3*t);                   % 제어입력 u(t)

%% 방법A : LDOB 코드 (test.m 과 동일)
z = -L*M*v(1); u_prev = 0; dA = zeros(1,N);   % d_hat(0)=0 으로 정렬 (Q-filter 영상태와 일치)
for k = 1:N
    d_hat = z + L*M*v(k);              % d_hat = z + L*M*v
    dA(k) = d_hat;
    z_dot = -L*(d_hat + M*u_prev);     % z_dot = -L*(d_hat + M*u_prev)
    z     = z + z_dot*dt;
    u_prev = u(k);                     % 이전 제어입력
end

%% 방법B : Q-filter 전달함수  d_hat = Q*(Pn^-1*v - u),  Pn^-1*v = M*vdot
e  = M*vdot - u;                       % 등가 총입력 - 실제입력 = 외란
dB = zeros(1,N); y = 0;
for k = 1:N
    dB(k) = y;
    y = y + dt*L*(e(k) - y);           % ydot = L(e - y)  =>  Q = L/(s+L)
end

%% 비교
err = dA - dB;
fprintf('max|dA-dB| = %.4e\n', max(abs(err)));
fprintf('rms(dA-dB) = %.4e\n', sqrt(mean(err.^2)));
fprintf('max|dA|    = %.4e  (relative max err = %.3e)\n', ...
        max(abs(dA)), max(abs(err))/max(abs(dA)));

%% plot
figure('Name','LDOB vs Q-filter');
subplot(2,1,1); hold on; grid on;
plot(t, dA, 'b','LineWidth',1.3);
plot(t, dB, 'r--','LineWidth',1.3);
ylabel('d\_hat'); legend({'A: LDOB code','B: Q-filter t.f.'},'Location','best');
title('LDOB code  vs  Q-filter transfer function');

subplot(2,1,2); grid on;
plot(t, err, 'k','LineWidth',1.0);
ylabel('A - B'); xlabel('time [s]'); title('difference');

saveas(gcf,'verify_ldob.png');
fprintf('saved: verify_ldob.png\n');
