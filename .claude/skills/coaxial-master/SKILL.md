---
name: coaxial-master
description: HOCBF 기반 동축반전로터 드론 MATLAB 시뮬레이션 전문 어시스턴트. 동축반전 드론 제어기 설계, 파라미터 튜닝, 시뮬레이션 코드 수정/디버깅, 결과 해석을 돕는다. Control Barrier Function, Backstepping, MPC, 비선형 외란관측기 관련 MATLAB 작업에 사용한다.
---

# Coaxial Rotor Drone Simulation — Domain Knowledge

## 연구 배경
박준성, 김형근 (인천대학교 기계공학과)의 논문:
"안전 제약조건을 만족하는 제어장벽함수 기반 동축반전로터 드론자세 제어"
(Attitude Control for Coaxial Rotor Drone using Control Barrier Function)

**핵심 기여:**
- 동축반전 로터 드론 + HOCBF(High-Order Control Barrier Function) 결합
- 모델 불확실성 환경에서 실시간 안전 제어 달성
- MPC 대비 최대 계산시간 1/10 수준으로 제약 위반 최소화

---

## 이론 핵심 개념

### 안전성 vs 안정성
| 개념 | 정의 | 수학적 표현 |
|------|------|-------------|
| 안전성 (Safety) | 위험 상태로 이탈하지 않음 | 전방 불변성(Forward Invariance) |
| 안정성 (Stability) | 목표 상태로 수렴 | 점근 안정성(Asymptotic Stability) |

### 안전 집합 (Safe Set)
```
C = {x ∈ ℝⁿ : b(x) ≥ 0}
b(x) ≥ 0 이면 안전한 상태
```

### CBF → HOCBF 확장
CBF는 1차 미분에만 의존 → 각도처럼 상대 차수 2 이상인 시스템에 한계
HOCBF는 연속 함수 ψᵢ를 정의하여 해결:
```
ψᵢ = ψ̇ᵢ₋₁ + αᵢ(ψᵢ₋₁) ≥ 0,  i = 1,...,m
αᵢ(·) = pᵢ(·)  (선형 Class K 함수 사용)
```

### 제어기 설계 흐름
```
각도 장벽 후보함수:  b₁ = Θ_max∘Θ_max - Θ∘Θ
  → ψ₁ = ḃ₁ + p₁b₁ = -2Θ∘Θ̇ + p₁b₁ ≥ 0
  → ψ₂ = ψ̇₁ + p₂ψ₁ ≥ 0  →  토크 상한 B₁ 도출

각속도 장벽 후보함수: b₂ = ω_max∘ω_max - ω∘ω
  → ψ₃ = ḃ₂ + p₃b₂ = -2ω∘(Tr/I) + p₃b₂ ≥ 0  →  토크 상한 B₂ 도출

QP:  Tr = argmin (1/2)||Tr - Tn||²
         s.t. Tr ≤ B₁,  Tr ≤ B₂
```
여기서 `∘`는 아다마르 곱 → MATLAB에서 `.*`

---

## 동역학 모델

### 좌표계: NED (North-East-Down)
- x: 북쪽(+), y: 동쪽(+), z: 아래쪽(+) → **고도는 음수**
- 롤(φ), 피치(θ), 요(ψ)

### 강체 동역학 (sixdof.m)
```
mẌ = R(Θ)Tt + mg + Dx
Iω̇ = Tr - ω×Iω + Dφ
```

### 동축반전 추력/모멘트 (CoaX_Dyn4.m)
```
Tz = -k₁Ω₁² - k₂cos(α)cos(β)Ω₂²   % 수직 추력
Tx = -k₂Ω₂²cos(α)sin(β)             % 수평 x
Ty = k₂Ω₂²sin(α)                    % 수평 y

τφ =  d·Ty                           % 롤 모멘트
τθ = -d·Tx                           % 피치 모멘트
τψ = γ₁Ω₁² - γ₂Ω₂²                 % 요 모멘트
```
**핵심:** Tx, Ty는 Ω₂와 스와시 플레이트 각(α,β)의 곱 → 최대 제어 모멘트가 Ω₂에 따라 실시간 변동

### 핵심 파라미터 (실제 시뮬 값)
```matlab
mass = 11.5;                          % kg
I = diag([0.2203, 0.2567, 0.1056]);  % kg·m²
D = 0.88;                             % 로터 직경 [m]
d = 0.2;                              % 하단로터~CoG 거리 [m]
rho = 1.225;                          % 공기밀도 [kg/m³]
cT = 0.091659;                        % 무차원 추력 계수
cQ = 0.003523;                        % 무차원 모멘트 계수

angle_max   = 2  * pi/180;           % 자세각 제한 [rad]
omega_max   = 5  * pi/180;           % 각속도 제한 [rad/s]
alpha_max   = 14 * pi/180;           % 스와시 플레이트 제한 [rad]
Omega_min   = 50;                     % 로터 최소 각속도 [rad/s]
Omega_max   = 100;                    % 로터 최대 각속도 [rad/s]
```

---

## 디렉토리 구조 및 파일 역할

```
시뮬/
├── main.m                              ← 메인 루프 (시뮬 전체 제어, 400Hz)
├── run_simulation.m                    ← GUI/파라미터 래퍼
├── DroneSimGUI.m                       ← App Designer GUI
├── get_default_params.m                ← 파라미터 중앙 저장소
│
├── Controller/
│   ├── BACKHOCBFQP.m                  ← 핵심: Backstepping + HOCBF + QP
│   ├── BACKHOCBFQP2~6.m              ← 파라미터 튜닝 변형들
│   ├── BACKDOB.m                      ← Backstepping + DOB (barrier 없음)
│   └── MPCctrl.m                      ← MPC 래퍼 (mpcmove 사용)
│
├── Dynamics/
│   ├── CoaX_Dyn4.m                    ← 추력/모멘트 계산 (Ω,α,β → F,τ)
│   ├── sixdof.m                        ← 6DOF Euler 적분
│   ├── TorqueMax.m                    ← 현재 상태의 최대 토크 계산
│   └── Quad_Dyn.m                     ← 쿼드로터 동역학 (비교용)
│
├── Trajectory/
│   ├── POS2THR.m                      ← 위치 PID → 가속도/추력 명령
│   └── POS2ATT.m                      ← 가속도 명령 → 자세 명령
│
├── Scenario/
│   ├── Scenario0.m                    ← 호버링 (0,0,-1)
│   ├── Scenario1~9.m                  ← 다양한 경로점 시나리오
│   └── waypoint.m / waypoint2~11.m   ← 경로점 순차 추종
│
├── LowLevelController/
│   └── compute_rotor_speed_CoaX.m   ← 역변환: (Tz,τ) → (Ω₁,Ω₂,α,β)
│
├── Environment/
│   ├── DisGen.m                       ← 외란 생성 (Mode 0~4)
│   └── NDOB.m                         ← 비선형 외란 관측기
│
└── Plot/
    ├── CoaX_Ani_Topview.m            ← 2D 상공 애니메이션
    ├── CoaX_Ani_tracking.m           ← 3D 추적 애니메이션
    └── plot_compare_anything.m       ← 다중 데이터셋 비교
```

---

## 상태벡터 인덱스 (main.m 기준)
```matlab
% state(1:16)
state(1)  = x;         % 북쪽 위치 [m]
state(2)  = y;         % 동쪽 위치 [m]
state(3)  = z;         % 고도 (NED, 음수가 위) [m]
state(4)  = phi;       % 롤각 [rad]
state(5)  = theta;     % 피치각 [rad]
state(6)  = psi;       % 요각 [rad]
state(7)  = u;         % 북쪽 속도 [m/s]
state(8)  = v;         % 동쪽 속도 [m/s]
state(9)  = w;         % 하향 속도 [m/s]
state(10) = p;         % 롤 각속도 [rad/s]
state(11) = q;         % 피치 각속도 [rad/s]
state(12) = r;         % 요 각속도 [rad/s]
state(13) = alpha;     % 스와시 플레이트 롤각 [rad]
state(14) = beta;      % 스와시 플레이트 피치각 [rad]
state(15) = Omega_up;  % 상단 로터 각속도 [rad/s]
state(16) = Omega_dw;  % 하단 로터 각속도 [rad/s]
```

---

## 제어기 입출력 인터페이스

### BACKHOCBFQP.m (HOCBF 핵심 제어기)
```matlab
function [U, x_upper] = BACKHOCBFQP(state_att, state_dot, state_des, ...
                                      state_max, P1, P2, P3, tau_filter, I, U_prev)
% 입력:
%   state_att  [3×1] : 현재 자세각 [φ,θ,ψ]
%   state_dot  [3×1] : 현재 각속도 [p,q,r]
%   state_des  [6×1] : 목표 자세 및 각속도 [φd,θd,ψd,pd,qd,rd]
%   state_max  [2×1] : [angle_max; omega_max]
%   P1,P2,P3  [3×3] : Barrier 파라미터 행렬 (양의 정부호)
%   tau_filter       : 저역통과 필터 시정수 (기본 0.02s)
%   I          [3×3] : 관성 모멘트 행렬
%   U_prev     [3×1] : 이전 제어 토크 (필터용)
% 출력:
%   U         [3×1] : 최적 제어 토크 [τφ,τθ,τψ]
%   x_upper   [6×1] : Barrier 상한 [B1_φ,B1_θ,B1_ψ, B2_φ,B2_θ,B2_ψ]
```

### NDOB.m (외란 관측기)
```matlab
function [d_hat, z_new] = NDOB(omega, tau_d, z, L, I)
% L: 관측기 이득 (스칼라, 기본값 1)
% d_hat: 추정된 외란 토크 [3×1]
```

---

## 공통 작업 가이드

### 1. 새 시나리오 추가
```matlab
% Scenario/ScenarioN.m 템플릿
function [waypoints, t_wp] = ScenarioN()
    % waypoints: [4×K] = [x; y; z; yaw] (NED, 고도는 음수)
    waypoints = [0, 5,  5, 0;
                 0, 0,  5, 5;
                -1,-2, -1,-1;   % z: NED, -1 = 고도 1m
                 0, 0,  0, 0];
    t_wp = [5, 10, 15, 20];
end
```
main.m 상단에서 `Scenario = N;` 으로 선택

### 2. HOCBF 파라미터 튜닝 (p1, p2, p3)

**문제 증상별 조정:**
| 증상 | 원인 | 조정 |
|------|------|------|
| QP infeasible | p1/p2 너무 크거나 초기조건 불량 | p1 줄이기 |
| 제약 위반 발생 | p1/p2 너무 작음 | p1 늘리기 |
| 채터링 | 필터 시정수 작음 | tau_filter 늘리기 (0.02→0.05) |
| 수렴 느림 | K1/K2 gain 작음 | K1, K2 늘리기 |

```matlab
% 권장 초기값
P1 = 100 * eye(3);    % 각도 barrier 1차
P2 = 1   * eye(3);    % 각도 barrier 2차
P3 = 10  * eye(3);    % 각속도 barrier
```

### 3. 외란 설정 (DisGen.m)
```matlab
dis_mode = 2;             % 0:없음, 1:상수, 2:정현파, 3:저주파, 4:고주파
dis_moment_max = 0.025;  % 최대 토크 외란 [Nm] (최대 모멘트의 ~5%)
```

### 4. HOCBF vs MPC 비교 실험
```matlab
controller_mode = 'BACKHOCBFQP';  % or 'MPC'
% MPC 추천 지평선 (논문 기준): Np=40, Nc=8
```

### 5. 새 컨트롤러 변형 추가
기존 BACKHOCBFQP.m을 복사 → BACKHOCBFQPN.m으로 저장 후 수정.
**모든 함수는 반드시 분리된 .m 파일로 관리** (main.m 인라인 금지).

---

## 디버깅 가이드

### QP Infeasible
```
Error using quadprog: No feasible point found.
```
1. 현재 상태가 이미 제약 위반인지 확인: `disp(abs([phi theta psi]) * 180/pi)`
2. TorqueMax.m으로 현재 최대 토크 확인
3. P1 값 절반으로 줄이기 (100 → 50)

### 채터링
```matlab
tau_filter = 0.05;  % 0.02 → 늘리면 부드러워짐 (응답속도 트레이드오프)
```

### 외란 관측기 발산
```matlab
L = 0.5;  % 1 → 줄이면 안정적, 추정 속도 느려짐
```

### 수직 추력 부족 (드론이 떨어짐)
- NED: z 아래가 양수 → 상승하려면 `Tz < -m*g` (음수) 확인
- compute_rotor_speed_CoaX.m 내 최소 추력 안전값(기본 10N) 확인

---

## 결과 분석

### Barrier Violation 합계 계산
```matlab
% 논문 Table 3 재현
violation_angle = sum(max(0, abs(phi_log) - angle_max)) * dt_ctrl;  % [deg²]
violation_omega = sum(max(0, abs(p_log)   - omega_max)) * dt_ctrl;  % [deg²/s²]
```

### TPR (Thrust-to-Power Ratio) 계산
```matlab
% 논문 식 (29)
T = sum(cT .* rho .* pi .* D^4 .* Omega.^2);
P = sum(2*pi .* cQ .* rho .* D^5 .* Omega.^3);
eta = T / P;
```

---

## 코딩 스타일 규칙

1. **함수 분리 원칙:** 각 기능은 독립적인 .m 파일로 분리 (main.m에 인라인 금지)
2. **NED 부호 주의:** 고도 명령은 항상 음수 (`z_d = -1` → 1m 고도)
3. **아다마르 곱:** 수식의 `∘` 기호는 MATLAB에서 `.*`
4. **단위 통일:** 각도는 rad 사용 (출력/표시 시에만 deg 변환)
5. **파라미터 중앙화:** 새 파라미터는 `get_default_params.m`에 추가
6. **제어 주기:** dt_ctrl = 0.0025s (400Hz), 시뮬 적분: dt_sim = 0.0001s
