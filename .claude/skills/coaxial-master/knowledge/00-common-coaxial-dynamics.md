# 동축반전 로터 드론 — 공통 동역학 지식

> 이 문서는 COAX-SIM 프로젝트가 사용하는 **동축반전(coaxial counter-rotating) 로터 드론**의 모델·좌표계·파라미터·계층 제어 구조에 대한 공통 지식을 정리한 것이다.
> 개별 논문 요약은 같은 폴더의 다른 .md 파일을 참조한다.

---

## 1. 동축반전 로터 드론 개요

### 1.1 형상
- **2 개의 로터가 동일 축(coaxial)** 상하로 배치, 서로 반대방향으로 회전 (counter-rotating)
- 상부 로터 (upper rotor, 첨자 `up` 또는 `1`)
- 하부 로터 (lower rotor, 첨자 `dw` 또는 `2`)
- 하부 로터는 **스와시 플레이트(swashplate)** 로 틸트 가능 (각도 α, β)

### 1.2 추진/제어 메커니즘
| 채널 | 메커니즘 |
|------|----------|
| **수직 추력 (Z, 추력)** | 상·하부 로터 속도 합 (Ω_up, Ω_dw)으로 조절 |
| **요(yaw, ψ) 토크** | 상·하부 로터 속도 **차이**에 의한 반동 토크 차로 조절 (counter-rotating의 핵심) |
| **롤(roll, φ) 토크** | 하부 로터 스와시 플레이트 α 틸트 → 하부 추력 방향이 기울어 모멘트 발생 |
| **피치(pitch, θ) 토크** | 하부 로터 스와시 플레이트 β 틸트 → 동일 원리 |

### 1.3 동축반전의 장점
- **요 모멘트가 능동적 제어 가능** (단일 로터 헬리콥터는 tail rotor 필요)
- 작은 footprint, 강한 풍속 저항 (twin rotor 효과)
- 호버 효율은 ducted 단일보다 낮음 (rotor 간 inflow 간섭)

### 1.4 단점
- **상·하부 로터 inflow 간섭**: 하부 로터가 상부 wake 안에서 작동 → 추력 손실 (보통 10~20 %)
- 메커니즘 복잡 (스와시 + 2모터)
- 모델 식별 어려움

---

## 2. 좌표계 및 부호 규약

### 2.1 NED (North-East-Down) 관성좌표계
- **x**: 북쪽 (+North)
- **y**: 동쪽 (+East)
- **z**: 아래쪽 (+Down) → **고도는 음수**, 1 m 고도 = `z = -1`
- 중력 가속도: `g = +9.81 m/s²` 그대로 z 방향 (아래)

### 2.2 body frame
- 원점: 드론 무게중심(CoG)
- x_b: 전방, y_b: 우측, z_b: 하방
- 회전순서: ZYX (yaw ψ → pitch θ → roll φ) — MATLAB `eul2rotm([psi theta phi])` 와 호환

### 2.3 오일러 각
- φ (roll), θ (pitch), ψ (yaw) [rad]
- 미분: `Theta_dot = pqr2deul(Theta) * omega_body`  ([sixdof.m:25-33](Dynamics/sixdof.m))
  ```
  R(1,:) = [1, sin(φ)tan(θ), cos(φ)tan(θ)]
  R(2,:) = [0,        cos(φ),       -sin(φ)]
  R(3,:) = [0, sin(φ)/cos(θ), cos(φ)/cos(θ)]
  ```
  주의: θ → 90° 부근에서 특이점 (gimbal lock)

---

## 3. 상태벡터 (COAX-SIM 관례)

```matlab
% state(1:16)
state(1)  = x;          % 북쪽 위치 [m]
state(2)  = y;          % 동쪽 위치 [m]
state(3)  = z;          % NED 고도 (음수가 위) [m]
state(4)  = phi;        % roll [rad]
state(5)  = theta;      % pitch [rad]
state(6)  = psi;        % yaw [rad]
state(7)  = u;          % body x 속도 [m/s]
state(8)  = v;          % body y 속도 [m/s]
state(9)  = w;          % body z 속도 [m/s]
state(10) = p;          % roll rate [rad/s]
state(11) = q;          % pitch rate [rad/s]
state(12) = r;          % yaw rate [rad/s]
state(13) = alpha;      % 스와시 roll 각 [rad]
state(14) = beta;       % 스와시 pitch 각 [rad]
state(15) = Omega_up;   % 상부 로터 각속도 [rad/s]
state(16) = Omega_dw;   % 하부 로터 각속도 [rad/s]
```

> 주의: `state(7:9)`는 **body frame** 속도 (`linvel`).  
> 위치 미분: `X_1dot(1:3) = eul2rotm(eul) * linvel` ([sixdof.m:10](Dynamics/sixdof.m))

---

## 4. 6DOF 강체 동역학

[sixdof.m](Dynamics/sixdof.m) 구현 기준 (body frame 속도 표현):

### 4.1 운동학 (Kinematics)
```
ẋ_NED   = R(Θ) · v_body                       % 위치
Θ̇       = T(Θ) · ω_body                       % 자세 (Euler rate)
```

### 4.2 동역학 (Dynamics, body frame)
```
v̇_body = (F_total) / m  -  ω × v_body         % 병진
ω̇       = I⁻¹ · ( τ_total  -  ω × (I·ω) )    % 회전
```
- `F_total = F_rotor + R(Θ)ᵀ · m·g·ẑ_NED + F_disturb_body`
- `τ_total = τ_rotor + τ_disturb`
- I = `diag([Ixx, Iyy, Izz])` (cross-product of inertia ≈ 0 가정)

### 4.3 적분
- Forward Euler: `state ← state + dt · state_1dot`
- 시뮬레이션 스텝 `dt_sim = 0.001 s` (1 kHz) — [main.m:33](main.m)

---

## 5. 동축반전 공력 모델

### 5.1 무차원 계수 → 차원 있는 형태
표준 표기 (revolutions/sec):
```
T = c_T · ρ · n² · D⁴            % 추력 [N]
Q = c_Q · ρ · n² · D⁵            % 토크 [N·m]
```
COAX-SIM 코드 표기 (Ω in rad/s):
```
T = k     · Ω²,   k     = c_T_hover · ρ · D⁴ / (2π)²
Q = γ     · Ω²,   γ     = c_Q_hover · ρ · D⁵ / (2π)²
```
([main.m:55-67](main.m))

### 5.2 상·하 로터 간섭 (interference)
[main.m:51-67](main.m):
```matlab
k_up = k_total / (1 + p.kT_interf);   % 분배 후 상부
k_dw = p.kT_interf * k_up;             % 하부는 kT_interf 배
% kT_interf = 0.85 → 하부 추력이 상부의 85% (15% 손실)
% kQ_interf = 0.90 → 하부 토크가 상부의 90% (10% 손실)
```
이는 하부 로터가 상부 wake 안에서 작동하기 때문이다.

### 5.3 추력/모멘트 식 ([CoaX_Dyn.m](Dynamics/CoaX_Dyn.m))
```matlab
T_up = c_T(1) · π · ρ · D⁴ · Ω_up²              % 상부 추력 (수직)
T_dw = c_T(2) · π · ρ · D⁴ · Ω_dw²              % 하부 추력 크기
Q_up = c_Q(1) · π · ρ · D⁵ · Ω_up²              % 상부 anti-torque
Q_dw = c_Q(2) · π · ρ · D⁵ · Ω_dw²              % 하부 anti-torque

% 하부 추력 벡터 (스와시 틸트 반영)
T_dw_body = R(α, β) · [0; 0; -T_dw]              % body frame

F_total = T_dw_body + [0; 0; -T_up]              % body frame, z 음수 = 추력 위쪽

% 모멘트
τ_φ = r_cd · T_dw · sin(α)                       % 롤
τ_θ = r_cd · T_dw · cos(α) · sin(β)              % 피치
τ_ψ = Q_up - Q_dw                                % 요 (counter-rotating 차)
```
여기서 `r_cd` = CoG ~ 하부 로터 거리 [m] (코드: `p.r_cd = 0.2`)

### 5.4 핵심 관찰: 제어 토크는 Ω_dw에 의존
- τ_φ, τ_θ ∝ T_dw ∝ Ω_dw² · sin(α 또는 β)
- → **현재 Ω_dw 값에 따라 최대 가능 토크가 시변(time-varying)** ([TorqueMax.m](Dynamics/TorqueMax.m))
  ```matlab
  Torque_max(1) = r_cd · k(2) · Ω_dw² · sin(α_max)    % roll
  Torque_max(2) = r_cd · k(2) · Ω_dw² · sin(β_max)    % pitch
  Torque_max(3) = 0.2                                  % yaw (hard limit)
  ```
- HOCBF 제약식에 이 시변 한계를 넣어야 정확함 (현재 구현은 `p.Omega_max` 기반 정적 한계)

---

## 6. 역변환: (Thrust, Torque) → (Ω_up, Ω_dw, α, β)

[compute_rotor_speed_CoaX.m](Controller/compute_rotor_speed_CoaX.m):

### 6.1 로터 속도 (Thrust + yaw로부터)
연립방정식:
```
T_d   = k_1 · Ω_up² + k_2 · Ω_dw²       (수직 추력)
τ_ψ_d = γ_1 · Ω_up² - γ_2 · Ω_dw²       (요 토크)
```
해:
```
Ω_up_d² = (-γ_2 · T_d + k_2 · τ_ψ_d) / (k_1·γ_2 + k_2·γ_1)
Ω_dw_d² = (-γ_1 · T_d - k_1 · τ_ψ_d) / (k_1·γ_2 + k_2·γ_1)
```
(T_d 는 NED z방향 양수가 아래 → `T_d < 0`)

### 6.2 스와시 각 (roll, pitch 토크로부터)
```
α_d = τ_φ_d / ( k_2 · Ω_dw² · r_cd )      % roll
β_d = τ_θ_d / ( k_2 · Ω_dw² · r_cd )      % pitch
```
(소각 근사: `sin(α) ≈ α`)

### 6.3 사후 포화
```matlab
Ω_up_d ∈ [0, Omega_max]
Ω_dw_d ∈ [0, Omega_max]
α_d    ∈ [-α_max, α_max]
β_d    ∈ [-β_max, β_max]
```
포화가 일어나면 실제 토크가 명령과 다름 → 안전제약 위반 가능성

---

## 7. 액추에이터 동역학

### 7.1 모드 1: instant (이상화)
- 명령 = 실제, lag 없음

### 7.2 모드 2: 1차 lag ([CoaX_ActDyn.m](Dynamics/CoaX_ActDyn.m))
```
ẋ = (x_cmd - x) / τ
```
- `τ_sw  = 0.05 s` (스와시 서보)
- `τ_rot = 0.1  s` (로터 모터)

### 7.3 모드 3: Schafroth 비선형 로터 + 1차 서보 lag
**로터** ([RotorDynSchafroth.m](Dynamics/RotorDynSchafroth.m)):
```
J_rot · Ω̇ = K_motor · (Ω_cmd - Ω) - γ · (Ω² - Ω_cmd²)
```
- 1항: 모터 토크 (선형)
- 2항: 공력 항력 토크 (Ω² 비례)
- 정상상태: `Ω_ss = Ω_cmd` (평형)
- 출처: Schafroth 2010 CEP

**서보** ([ServoActDyn.m](Dynamics/ServoActDyn.m)): 1차 lag 동일

### 7.4 ESC/서보 주기 (이산)
[main.m:33-37](main.m):
```matlab
p.dt_sim   = 0.001;   % 적분    (1 kHz)
p.dt_ctrl  = 0.002;   % 제어    (500 Hz)
p.dt_rot   = 0.005;   % ESC     (200 Hz)
p.dt_servo = 0.02;    % 서보    ( 50 Hz)
```
**다중 시간척도** 시뮬레이션: 제어 → 액추에이터 명령은 서보/ESC 주기에 맞춰 ZOH

---

## 8. 외란 모델 ([DisGen.m](Dynamics/DisGen.m))

| `dis_mode` | 형태 | 용도 |
|------------|------|------|
| 0 | 없음 | baseline |
| 1 | 상수 `Dis_max` | bias 형 외란 |
| 2 | `Dis_max · sin(t)` | 1 rad/s 정현파 |
| 3 | `Dis_max · sin(0.1·t)` | 저주파 (gust 모사) |
| 4 | `Dis_max · sin(10·t)` | 고주파 (진동 모사) |

```matlab
p.Dis_max = [0.1; 0.02];   % [N (힘), N·m (모멘트)]
```
- 힘: NED 3축 동일
- 모멘트: body 3축 동일

---

## 9. 외란 관측기 (DOB)

### 9.1 LDOB (Linear Disturbance Observer, [LDOB.m](Controller/LDOB.m))
연속시간 전달함수:
```
d̂ / d  =  L / (s + L)              % 1차 저역통과
```
이산 구현 (auxiliary variable trick):
```matlab
p_aux = L · I · ω                   % auxiliary state
d_hat = z + p_aux                   % 외란 추정치
ż     = -L · (d_hat + τ_d)          % 내부 상태 업데이트
```
- 이득 `L`이 크면 빠르지만 노이즈 증폭
- COAX-SIM 기본값: `L_dob = 0.9` (자세), `L_dob_pos = 0.9` (위치)

### 9.2 위치 LDOB (LDOB_pos.m)
- 가속도 측정 또는 추정에서 외란 힘 d_F 복원
- POS2THR에 `d_hat_f` 로 들어가서 위치제어 보상 ([POS2THR.m:34](Controller/POS2THR.m))

### 9.3 비교 (논문 참조)
- NDOB (nonlinear DOB) — Schafroth 등
- ESO (Extended State Observer) — ISAT.16 논문
- TDE (Time-Delay Estimation) — Glida 2023 (ISAT.23)

→ 각 관측기는 knowledge 폴더의 해당 논문 요약 참조

---

## 10. 안전 제약 (CBF용)

### 10.1 자세각/각속도 한계 ([main.m:84-89](main.m))
```matlab
p.angle_max     = [5°; 5°; 360°];     % φ, θ는 ±5°, ψ는 무제한
p.angle_dot_max = [10°/s; 10°/s; 30°/s];
p.tilt_max      = [15°; 15°];          % α, β 스와시 한계
p.Omega_max     = 3000 rpm → ≈ 314 rad/s
```

### 10.2 yaw 채널 제외
- yaw 각 제한은 비물리적이므로 BACKHOCBFQP에서 강제 비활성:
  ```matlab
  A1_cond(3) = 0;
  B1_cond(3) = 1;                    % 항상 0 ≤ 1 → 무조건 만족
  ```
  ([BACKHOCBFQP.m:69-70](Controller/BACKHOCBFQP.m))

### 10.3 토크 포화 (QP `lb`, `ub`)
```matlab
lb = -Torque_max;
ub =  Torque_max;
```
`Torque_max`는 본래 [TorqueMax.m](Dynamics/TorqueMax.m)로 시변 계산 가능하나, 현재 호출 위치에 따라 정적 값 사용

---

## 11. 계층 제어 구조 (Hierarchical Control)

COAX-SIM 전체 흐름:

```
[목표 위치] pos_d
    │
    ▼
┌─────────────────────────────────────────────┐
│ 외부 루프 (Outer): POS2THR.m + LDOB_pos     │
│  - PD 위치제어 + d_hat_f 보상                │
│  - 출력: Acc_d (NED), Thrust_d              │
└─────────────────────────────────────────────┘
    │ Acc_d, Thrust_d
    ▼
┌─────────────────────────────────────────────┐
│ 중간 변환: ACC2ATT.m                         │
│  - Acc_d + ψ_d → φ_d, θ_d  (역기구학)       │
└─────────────────────────────────────────────┘
    │ [φ_d, θ_d, ψ_d]
    ▼
┌─────────────────────────────────────────────┐
│ 내부 루프 (Inner): 제어모드 선택              │
│  1: BACKDOB           (BSC + DOB)            │
│  2: BACKHOCBFQP       (BSC + CBF + QP)       │
│  3: BACKHOCBFQPDOB    (BSC + CBF + QP + DOB) │
│  4: MPCctrl           (Model Predictive)     │
│  출력: τ_d (3축 토크 명령)                    │
└─────────────────────────────────────────────┘
    │ τ_d, Thrust_d
    ▼
┌─────────────────────────────────────────────┐
│ Low-level: compute_rotor_speed_CoaX.m       │
│  - 역변환: → (Ω_up_d, Ω_dw_d, α_d, β_d)     │
└─────────────────────────────────────────────┘
    │
    ▼
┌─────────────────────────────────────────────┐
│ 액추에이터 동역학 (rotor_dyn_mode 1/2/3)     │
│  - CoaX_ActDyn / RotorDynSchafroth 등        │
└─────────────────────────────────────────────┘
    │ (Ω_up, Ω_dw, α, β)
    ▼
┌─────────────────────────────────────────────┐
│ CoaX_Dyn.m → (F, τ_body)                    │
│ sixdof.m   → 상태 업데이트                   │
└─────────────────────────────────────────────┘
```

**Time-scale separation**:
- 위치 루프: ~ 1 Hz 대역폭
- 자세 루프: ~ 10 Hz 대역폭
- 로터 ESC: ~ 50 Hz 대역폭

내부 루프가 외부보다 충분히 빠를 때 분리 설계 정당화 (singular perturbation argument).

---

## 12. Backstepping (자세 제어) 표준 형태

[BACKDOB.m](Controller/BACKDOB.m), [BACKHOCBFQP.m](Controller/BACKHOCBFQP.m) 공통:

```
% Step 1: 자세 오차
Z₁ = Θ - Θ_d

% Step 2: 가상 입력 (각속도 명령)
θ_2d = Θ̇_d - K₁ · Z₁

% Step 3: 각속도 오차
Z₂ = Θ̇ - θ_2d

% Step 4: Lyapunov V = ½(Z₁ᵀZ₁ + Z₂ᵀZ₂)
%         V̇ ≤ 0 보장하는 토크
τ_nom = -I · (Z₁ - Θ̈_2d + K₂ · Z₂)  -  d̂      % DOB 보상 포함
```

- `K₁`: 외측 (각도 오차 수렴)
- `K₂`: 내측 (각속도 오차 수렴)
- 코드 기본: `K1_bsc = [15;15;15]`, `K2_bsc = [5;5;5]`

→ Backstepping 일반 이론은 `cep-2014-hierarchical-backstepping-mav.md` (CEP 2014 논문) 참조

---

## 13. HOCBF (High-Order Control Barrier Function) 핵심

자세각의 상대차수가 2 (각도 → 각속도 → 토크)이므로 일반 CBF로는 부족. HOCBF로 확장:

### 13.1 안전 집합
```
C = { x : b(x) ≥ 0 }
b₁ = Θ_max² - Θ²        % 자세각 제한
b₂ = Θ̇_max² - Θ̇²       % 각속도 제한
```

### 13.2 HOCBF 시퀀스
```
ψ₀ = b₁
ψ₁ = ψ̇₀ + P₁ · ψ₀  =  -2Θ·Θ̇ + P₁ · b₁
ψ₂ = ψ̇₁ + P₂ · ψ₁  ≥ 0    ← 제어입력(τ)이 처음 등장하는 차수
```

### 13.3 자세각 제약 (B₁ → τ 상한)
ψ₂ ≥ 0 전개:
```
I⁻¹ · diag(2Θ) · τ  ≤  -2Θ̇² + (P₁+P₂)·(-2Θ·Θ̇) + P₁·P₂·b₁

→ A₁ · τ ≤ B₁_cond     (요소별)
  A₁ = 2Θ
  B₁ = I·(-2Θ̇² + (P₁+P₂)·(-2Θ·Θ̇) + P₁·P₂·b₁)
```

### 13.4 각속도 제약 (B₂ → τ 상한)
b₂는 상대차수 1 → 일반 CBF로 충분:
```
ψ₃ = ḃ₂ + P₃ · b₂  =  -2Θ̇·I⁻¹·τ + P₃·b₂  ≥ 0
A₂ = 2Θ̇,   B₂ = I · P₃ · b₂
```

### 13.5 QP 정식화
```
minimize    ½ ‖τ - τ_filt‖²
subject to  A₁ τ ≤ B₁_cond            % 자세각 안전
            A₂ τ ≤ B₂_cond            % 각속도 안전
            -τ_max ≤ τ ≤ τ_max        % 액추에이터 한계
```
- `τ_filt`: nominal 토크의 1차 LPF (`τ_filt = 0.02 s`) → chattering 억제
- MATLAB: `quadprog`
- DOB 결합 시 ([BACKHOCBFQPDOB.m:48-50](Controller/BACKHOCBFQPDOB.m)):
  ```matlab
  B1_cond = (...) - 2*Θ.*d̂
  B2_cond = (...) - 2*Θ̇.*d̂
  ```
  (외란이 안전 제약을 좁힌다)

→ 자세한 유도·표는 `park-kim-hocbf-coaxial-attitude.md` 참조

---

## 14. 핵심 파라미터 (COAX-SIM 기본값)

### 14.1 기체
| 파라미터 | 값 | 단위 | 출처 |
|----------|----|----|------|
| `Mass`   | 11.5 | kg | main.m |
| `Ixx`    | 0.2203 | kg·m² | main.m |
| `Iyy`    | 0.2567 | kg·m² | main.m |
| `Izz`    | 0.1056 | kg·m² | main.m |
| `r_cd`   | 0.2 | m | CoG ~ 하부 로터 |
| `D`      | 0.88 | m | 로터 직경 |
| `rho`    | 1.225 | kg/m³ | 공기 밀도 (해수면) |

### 14.2 공력
| 파라미터 | 값 | 비고 |
|----------|----|----|
| `cT_hover` | 0.091659 | 무차원 |
| `cQ_hover` | 0.003523 | 무차원 |
| `kT_interf` | 0.85 | 하부/상부 추력비 |
| `kQ_interf` | 0.90 | 하부/상부 토크비 |

### 14.3 제약
| 파라미터 | 값 | 비고 |
|----------|----|----|
| `angle_max(φ,θ)`     | 5° (≈ 0.0873 rad)  | 자세 한계 |
| `angle_max(ψ)`       | 360° | yaw 무제한 |
| `angle_dot_max(φ,θ)` | 10°/s | 각속도 한계 |
| `angle_dot_max(ψ)`   | 30°/s |  |
| `tilt_max(α,β)`      | 15° | 스와시 |
| `Omega_max`          | 3000 rpm ≈ 314 rad/s | 로터 |

### 14.4 액추에이터
| 파라미터 | 값 | 비고 |
|----------|----|----|
| `tau_sw`  | 0.05 s | 서보 시정수 |
| `tau_rot` | 0.10 s | 모터 시정수 |
| `J_rot`   | 0.01 kg·m² | 로터 관성 (Schafroth 모드) |
| `K_motor` | `J_rot/tau_rot` | 일관성 |

### 14.5 제어기
| 그룹 | 게인 | 값 |
|------|------|----|
| 위치 BSC | K1_pos / K2_pos | [1;1;1] / [3;3;3] |
| 자세 BSC | K1_bsc / K2_bsc | [15;15;15] / [5;5;5] |
| HOCBF    | P1 / P2 / P3 | 10 / 10 / 10 |
| 필터     | tau_filt | 0.02 s |

### 14.6 관측기
| 파라미터 | 값 |
|----------|----|
| `L_dob`     | 0.9 (자세) |
| `L_dob_pos` | 0.9 (위치) |

### 14.7 센서 노이즈 (obs_mode 3, 3σ ≈ max)
| 측정 | σ | 약 1σ |
|------|---|-------|
| 위치  | 0.003 m   | 1 cm |
| 속도  | 0.003 m/s | 1 cm/s |
| 가속도| 0.003 m/s² | 0.03 m/s² |
| 자세  | 0.03° | 0.1° |
| 각속도| 0.03°/s | 0.1°/s |

---

## 15. 단위·표기 규약

| 양 | 단위 |
|----|----|
| 길이 | m |
| 각도 | rad (출력 시만 deg 변환) |
| 각속도 | rad/s |
| 시간 | s |
| 추력 / 힘 | N (양수: body z 음 = 위쪽) |
| 토크 | N·m |
| 로터 속도 Ω | rad/s |

- 수식 내 `∘` (아다마르 곱) → MATLAB `.*`
- 수식 내 `I` 와 `J`: 본 프로젝트는 `Inertia` = `I` 통일

---

## 16. 모드 선택 매트릭스 ([main.m:22-26](main.m))

```matlab
p.scenario_id    = 3;   % 1: box+yaw  2: circle R=5  3: circle+heading  4: RRT CSV
p.ctrl_mode      = 3;   % 1: BSC+DOB  2: BSC+CBF  3: BSC+CBF+DOB  4: MPC
p.rotor_dyn_mode = 3;   % 1: instant  2: 1차 lag  3: Schafroth rotor+servo lag
p.dis_mode       = 3;   % 0: off  1: const  2: sin(t)  3: sin(0.1t)  4: sin(10t)
p.obs_mode       = 1;   % 1: true state+true dis  2: noisy+LDOB
```

**조합 권장:**
| 목적 | scenario | ctrl | rotor | dis | obs |
|------|----------|------|-------|-----|-----|
| 기본 호버 확인 | 0 | 1 | 1 | 0 | 1 |
| CBF baseline | 3 | 2 | 1 | 0 | 1 |
| CBF + 외란 | 3 | 3 | 2 | 3 | 2 |
| 가장 현실적 | 3 | 3 | 3 | 3 | 2 |

---

## 17. 디버깅 체크리스트

### 17.1 드론이 떨어짐 (Z 발산)
- NED 부호: `Thrust_d < 0` 인지 확인 (양수면 아래로 미는 것)
- `compute_rotor_speed_CoaX` 의 `sqrt(...)` 인자가 음수면 `NaN` → Ω = 0
- 호버 추력: `T_hover = Mass * g = 11.5 * 9.81 ≈ 113 N` 필요

### 17.2 자세 발산
- `K1_bsc`/`K2_bsc` 부호 (양수)
- `Inertia` 단위 (kg·m²)
- 적분 발산: `dt_sim` 줄이기 (1 kHz → 2 kHz)

### 17.3 QP infeasible (BACKHOCBFQP)
- 현재 상태가 이미 제약 위반인지 확인: `abs([phi theta psi])*180/pi > angle_max`
- P1·P2 곱이 너무 크면 좁아짐 → 절반으로 줄여보기
- yaw 채널: `A1_cond(3)=0; B1_cond(3)=1` 가 풀려있는지 확인

### 17.4 채터링
- `tau_filt` 0.02 → 0.05 늘리기
- HOCBF QP의 `f = -2 * Torque_filt_prev` 가 LPF 출력인지 확인

### 17.5 DOB 발산
- `L_dob` 작게 (0.9 → 0.3)
- `dt_ctrl` 줄이기
- omega 노이즈가 너무 크면 미분 효과 발산 가능 → 사전 LPF 권장

---

## 18. 성능 지표

### 18.1 안전 제약 위반 (논문 Table 3 식)
```matlab
% 자세각 위반 [deg²·s]
viol_angle = sum( max(0, abs(phi_log) - angle_max(1)).^2 ) * dt_ctrl * (180/pi)^2;

% 각속도 위반 [deg²/s²·s]
viol_omega = sum( max(0, abs(p_log) - omega_max(1)).^2 ) * dt_ctrl * (180/pi)^2;
```

### 18.2 추적 오차
```matlab
% RMSE (NED 3축 합)
e_pos = sqrt( mean( sum((pos - pos_d).^2, 1) ) );
```

### 18.3 TPR (Thrust-to-Power Ratio, 동축반전 효율)
```matlab
T = sum( cT * rho * pi * D^4 * Omega.^2 );
P = sum( 2*pi * cQ * rho * D^5 * Omega.^3 );
eta = T / P;
```

### 18.4 제어기 계산시간
```matlab
tic; [tau, ~, ~] = BACKHOCBFQP(...); t_qp = toc;
% MPC 대비 1/10 수준이어야 (논문 결과)
```

---

## 19. 좌표·부호 함정 (Lessons Learned)

| 함정 | 결과 | 대처 |
|------|------|------|
| NED z를 ENU처럼 +로 처리 | 드론이 추락 | 모든 `pos(3)`, `Thrust_d` 음수 관례 확인 |
| `eul2rotm` 인자 순서 ([6,5,4] vs [4,5,6]) | 회전 잘못 | 코드는 `[psi, theta, phi]` = `[X(6),X(5),X(4)]` |
| `pqr2deul` 특이점 (θ ≈ ±90°) | 폭주 | 자세 한계로 회피 (angle_max 5°이면 안전) |
| QP의 `Theta_dot.^2` 누락 | 안전 마진 잘못 | HOCBF 식 (13.3) 그대로 |
| DOB가 controller 동작 전에 호출 → `Torque_d` 없음 | d_hat 부정확 | controller → DOB 순서 (이전 τ로 추정) |
| `compute_rotor_speed_CoaX` 에서 `Ω_dw² = (음수)` | NaN | T_d 부호와 yaw 토크 크기 점검 |

---

## 20. 논문 ↔ 코드 ↔ 본 문서 매핑

| 주제 | 본 문서 절 | 코드 | 논문 (knowledge/) |
|------|----------|------|------------------|
| 강체 6DOF | §4 | sixdof.m | (일반 항공공학) |
| 동축 공력 모델 | §5 | CoaX_Dyn.m | schafroth-2010-cep, schafroth-2010-jirs |
| 로터/서보 lag | §7 | RotorDynSchafroth.m, CoaX_ActDyn.m | schafroth-2010-cep |
| Backstepping | §12 | BACKDOB.m | cep-2014-hierarchical-backstepping-mav |
| HOCBF + QP | §13 | BACKHOCBFQP.m | park-kim-hocbf-coaxial-attitude |
| DOB 자세 | §9 | LDOB.m | isat-2017-dob-hierarchical-coaxial |
| ESO 비교 | (§9) | — | isat-2016-eso-coaxial-uav |
| TDE / 퍼지 | — | — | isat-2023-tde-fuzzy-glida |
| 위치-자세 계층 | §11 | POS2THR.m, ACC2ATT.m | isat-2017-dob-hierarchical-coaxial, cep-2014-hierarchical-backstepping-mav |

---

## 21. 참고

- 본 문서의 수식·관례는 코드(2026-05 시점) 상태 기준
- 코드가 변경되면 §3 상태벡터, §14 파라미터, §16 모드 매트릭스가 가장 빨리 stale 될 수 있음
- 각 논문 요약은 같은 폴더 별도 .md 파일 참조
- 인덱스: `README.md`
