# Schafroth 2010 (JIRS) — muFly 마이크로헬리콥터 모델링·시스템 식별

## 메타데이터
- **저자:** Dario Schafroth, Christian Bermes, Samir Bouabdallah, Roland Siegwart
- **소속:** Autonomous System Lab, Swiss Federal Institute of Technology (ETHZ), Zurich
- **저널:** Journal of Intelligent and Robotic Systems (JIRS), Vol. 57, pp. 27–47
- **DOI:** 10.1007/s10846-009-9379-x
- **수신/게재:** 2009-02-01 / 2009-08-01 / Online 2009-10-27, © Springer 2009 (저널 표지 2010)
- **프로젝트:** EU 6FP STREP muFly (FP6-IST-034120) — 자율 마이크로헬리콥터(감시·정찰)
- **시스템:** muFly micro coaxial helicopter (2nd prototype)

## 핵심 기여
- 동축 마이크로헬리콥터(muFly) 전용 **비선형 18-state full-rigid-body 모델** 도출
- 단순화/제어기 친화적이면서도 실제 하드웨어(electro motor, swash plate, **stabilizer bar**) 동역학을 모두 반영
- **Prediction Error Method (PEM)** 로 호버 부근 선형화 부분시스템(pitch/roll/heave/yaw)별 식별 절차 제시
- CAD/벤치 측정 + PEM 식별 + 호버 조건 매칭의 **하이브리드 파라미터 결정법** 정립
- 식별된 17개 파라미터 표(Table 1) 제공 — COAX-SIM 등 후속 시뮬레이터의 정량적 베이스라인
- CEP 2010 자매 논문 대비 **모델링·식별 절차 자체에 집중**하여 식 유도·전개를 보다 상세히 설명

## 시스템 개요
- **형상:** 17 cm 폭, 15 cm 높이 동축헬리콥터, 질량 95 g (Fig. 1)
- **로터:** 직경 ≈ 17.5 cm (R = 0.0875 m), 두 lightweight BLDC 모터로 역회전 구동, 기어비 1.5
- **조작:** Yaw = 두 로터 속도차, Heave(고도) = 두 로터 동시 가감속, Pitch/Roll = **하부 로터에만** 부착된 conventional swash plate(서보 2개)
- **수동안정화:** 상부 로터 위 **stabilizer bar(플라이바)** — 큰 관성으로 동체 roll/pitch에 뒤따라 cyclic pitch를 줘 TPP를 기울여 redress moment 생성 (Fig. 4)
- **센서:** IMU(자세), Ultrasonic(고도). 수평위치 카메라/레이저는 개발 중(본 논문 시점 식별 불가 → x/y 동역학은 future work)
- **신호 범위:** 서보 PPM `u_serv ∈ [-1, 1]`, 모터 `u_mot ∈ [0, 1]`

### muFly 메커니즘 그림 (Fig. 1, Fig. 2 요약)
- 상부에 stabilizer bar, 하부에 servo+swash plate
- Inertial Frame J = N-E-D, Body Frame B는 CoG, 항공우주 관습 (z = down)
- 호버 시 힘/모멘트: `T_up, T_dw` (위쪽), `Q_up, Q_dw` (yaw 반작용), `G` (중력 ↓), `W_hub` (downwash drag ↓)

## 동역학 모델

### 강체 회전·병진 운동 (Sec. 3.1)
좌표계: J(inertial NED), B(body, CoG).
변환행렬 (Euler ZYX, `cα=cosα, sα=sinα`):
```
A_BJ = [ cθcψ                cθsψ              -sθ
        -cφsψ+sφsθcψ         cφcψ+sφsθsψ       sφcθ
         sφsψ+cφsθcψ        -sφcψ+cφsθsψ       cφcθ ]     ... (2)

R_JB = [ 1  sφ·sθ/sθ   sφ·sθ/sθ   ; ...   (오일러각 → ωbody 변환)
         0  sφ        -sφ          ;
         0  sφ/sθ      sφ/sθ      ]                       ... (3)
```
> ⚠️ 논문 식 (3)은 표기 오류처럼 보이나, 표준 Euler-rate 변환임:
> `[φ̇; θ̇; ψ̇] = R_JB · [p; q; r]` (4).

Newton-Euler (body frame):
```
[u̇; v̇; ẇ] = (1/m)·F − [p;q;r] × [u;v;w]           ... (5)
[ṗ; q̇; ṙ] = I⁻¹ ( M − [p;q;r] × I·[p;q;r] )         ... (6)
```
`I = diag(Ixx, Iyy, Izz)` (대칭 형상 가정).

### 로터 공력 모델 (Sec. 3.2)
호버 평형:
```
T_up + T_dw = G + W_hub
Q_up + Q_dw = 0  (yaw 균형)                          (cf. Eq. 20)
```
**추력/토크 (BET 기반 호버 가정, 일정 c_T, c_Q):**
```
T_i = c_Ti · π · ρ · R⁴ · Ω_i²   = c_Ti · k_T · Ω_i²        ... (8)
Q_i = c_Qi · π · ρ · R⁵ · Ω_i²   = c_Qi · k_Q · Ω_i²        ... (9)
```
- 하부 로터는 상부 downwash 안에서 운영 → inflow 증가 → `c_T,dw < c_T,up`은 아니지만 식별 결과 `c_T,dw=0.0115 < c_T,up=0.0131` (Table 1).
- 호버 부근 가정으로 `c_T, c_Q`를 상수로 취급. 자유비행 fuselage drag/inflow 변화는 무시.

**Thrust 방향 (TPP tilt) — 동축헬기 조향의 핵심:**
```
n_Ti = [ cosα·sinβ ;  sinα ;  −cosα·cosβ ]            ... (10)
```
- 하부 로터: swash plate 입력으로 (α_dw, β_dw) 결정
- 상부 로터: stabilizer bar 응답으로 (α_up, β_up) 결정
- Torque 방향은 z축 (역회전: `n_Qup = [0;0;1]`, `n_Qdw = [0;0;-1]`)             ... (11)

### Fuselage Drag (downwash 반작용)
```
W_hub = [0; 0; W_hub]^T,  파라미터로 호버 매칭에 사용     ... (12)
```

### 중력
```
G = A_BJ·[0;0;mg] = mg·[−sinθ;  cosθ·sinφ;  cosθ·cosφ]^T  ... (13)
```

### 스와시플레이트 → TPP tilt → 모멘트 매핑 (Sec. 3.3)
**Stabilizer bar (상부 로터, 1차 lag):**
```
η̇_bar = (1/T_f,up)·(φ − η_bar)
ζ̇_bar = (1/T_f,up)·(θ − ζ_bar)                       ... (14)

α_up = l_up · (η_bar − φ)
β_up = l_up · (ζ_bar − θ)                            ... (15)
```
- `η_bar, ζ_bar`: stabilizer bar 자체 roll/pitch
- `l_up`: bar tilt → TPP tilt 변환 linkage factor (식별 0.83)

**Swash plate (하부 로터, 1차 lag, 서보+로터 lumped):**
```
α̇_dw = (1/T_f,dw)·(−l_dw·u_serv2·θ_SPmax − α_dw)
β̇_dw = (1/T_f,dw)·(−l_dw·u_serv1·θ_SPmax − β_dw)     ... (16)
```
- `θ_SPmax = 15°`, `l_dw = 0.41`

**힘/모멘트 (Eq. 7, Eq. 33–36):**
```
F = T_up + T_dw + G + W_hub                                          ... (7)
M = Q_up + Q_dw + r_Cup × T_up + r_Cdw × T_dw                        ... (7)

# 성분 전개 (z축 정렬 가정 r_Cup = [0;0;z_up], r_Cdw = [0;0;z_dw])
F_x = c_Tdw·k_T·Ω_dw²·cosα_dw·sinβ_dw − c_Tup·k_T·Ω_up²·sinβ_up − mg·sinθ
F_y = c_Tdw·k_T·Ω_dw²·sinα_dw          + c_Tup·k_T·Ω_up²·cosβ·sinα_up + mg·cosθ·sinφ
F_z = −c_Tdw·k_T·Ω_dw²·cosα_dw·cosβ_dw − c_Tup·k_T·Ω_up²·cosα·cosβ_up
      + W_hub + mg·cosθ·cosφ                                          ... (35)

M_x = −z_dw·c_Tdw·k_T·Ω_dw²·sinβ_dw    − z_up·c_Tup·k_T·Ω_up²·cosβ_up·sinα_up
M_y =  z_dw·c_Tdw·k_T·Ω_dw²·cosα_dw·sinα_dw − z_up·c_Tup·k_T·Ω_up²·sinβ_up
M_z = (c_Qup,rot + c_Qup,stab)·k_Q·Ω_up² − c_Qdw·k_Q·Ω_dw²            ... (36)
```

### 액추에이터 1차 lag — BLDC 모터 모델 (Sec. 3.4)
일반식:
```
J_mot·ω̇ = (κ_M·U − κ_M·κ_E·ω)/R_Ω − d_R·ω − M_L                    ... (17)
```
헬리콥터에 적용 (기어, 배터리, 외부 드래그 토크 포함):
```
J_drive·Ω̇_i = (κ_M·U_bat·u_mot,i − κ_M·κ_E·i_gear·Ω_i)/(i_gear·R_Ω)
             − d_R·Ω_i − c_Qi·k_Q·Ω_i² / (i_gear²·η_gear)            ... (18)
```
- `κ_E, κ_M`: 전기·기계 모터상수 (식별 0.0045, 0.0035)
- `d_R`: friction, `R_Ω`: armature 저항, `i_gear=1.5`, `η_gear=0.84`
- BLDC ESC가 RPM feedback이 없으므로 **모터 동역학을 모델 내에 명시 포함** → 외부 RPM 센서 불요

### 전체 state-space 형태 (Sec. 3.5)
```
x = [x, y, z, u, v, w, φ, θ, ψ, p, q, r, α_dw, β_dw, α_up, β_up, Ω_dw, Ω_up]^T  ... (19)
u = [u_mot,dw, u_mot,up, u_serv1, u_serv2]^T
```
- **18 states, 4 inputs**. 블록도 Fig. 6.

## 시스템 식별 (Sec. 4)

### 4.1 Mechanical Parameters
- 질량, swash plate max angle, 로터 직경, body inertia → CAD 또는 직접 측정
- 어려운 파라미터: 공력 계수, 모터 상수, stabilizer/swash 시정수 → 별도 식별 필요

### 4.2 Aerodynamic Parameters (Test bench)
- 동축 rotor test bench [11]에서 `c_Ti, c_Qi` 직접 측정 (각 로터 추력·토크 계측)
- Stabilizer bar 토크 계수 `c_Qup,stab` 와 fuselage drag `W_hub`는 호버 조건 매칭으로 보정:
```
T_up,hov + T_dw,hov = G + W_hub
Q_up,rot,hov + Q_up,stab,hov = Q_dw,hov                       ... (20)
```

### 4.3 Electro Motor (CEDRAT 측정 + LS)
- 외부 partner CEDRAT [12] 실험: 일정 전압, 외부 토크 변경하며 RPM·전류 측정
- 정상상태 `ω̇=0` Eq. (17)에 **Least-Square** 적용 → `κ_E, κ_M, d_R, R_Ω` 식별 (Fig. 7)

### 4.4 Prediction Error Method (Sec. 4.4)
- 호버 부근에서 **선형화** 후, 비행 데이터로 식별
- Loss function:
```
ε(t,Θ) = y(t) − ŷ(t|t−1, Θ)                                    ... (21)
R_N(Θ) = (1/N) Σ ε·ε^T                                         ... (22)
V_N(Θ) = det(R_N(Θ))   →   Θ_min = argmin V_N                  ... (23)
```
- x/y는 위치 센서 부재로 제외 → **14-state linearized model을 4개 subsystem**으로 분리:
  - **Pitch**: `x = [θ, q, β_dw, β_up]^T`, `u = u_serv1`
  - **Roll**: `x = [φ, p, α_dw, α_up]^T`, `u = u_serv2`
  - **Heave**: `x = [z, w, Ω_dw, Ω_up]^T`, `u = [u_mot,dw, u_mot,up]^T`
  - **Yaw**: `x = [ψ, r, Ω_dw, Ω_up]^T`, `u = [u_mot,dw, u_mot,up]^T`

**Pitch subsystem A 행렬 (Eq. 24):**
```
A_pitch = [ 0                           1   0                            0
            c_Tup·k_T·Ω²_up,0·z_up/Iyy  0   c_Tdw·k_T·Ω²_dw,0·z_dw/Iyy  −c_Tup·k_T·Ω²_up,0·z_up/Iyy
            0                           0  −1/T_f,dw                     0
           −1/T_f,up                    0   0                            1/T_f,up ]
B_pitch = [0; 0; l_dw·θ_SPmax/T_f,dw; 0]
```

**Heave subsystem (Eq. 25–26):**
```
A_heave = [ 0  1   0                              0
            0  0  −2c_Tdw·k_T·Ω_dw,0/m           −2c_Tup·k_T·Ω_up,0/m
            0  0   M_dw                           0
            0  0   0                              M_up ]
M_dw = (1/J_drive,dw)·(−d_R − 2c_Qdw·k_Q·Ω_dw,0/(η_gear·i_gear²) − κ_E·κ_M/R_Ω)
M_up = (1/J_drive,up)·(−d_R − 2c_Qup·k_Q·Ω_up,0/(η_gear·i_gear²) − κ_E·κ_M/R_Ω)
```

### 4.5 Data Generation
- 파일럿 + **PID stabilizer** + chirp signal 입력 → 광대역 자극 확보
- PEM 식별은 컨트롤러 영향과 독립 (입력 신호만 기록)
- 호버 부근 초기조건이 중요 → 초기상태·호버 입력 deviation도 튜닝 변수
- Heave/Yaw는 **두 극이 원점(critically stable)** → 식별 구간 짧게 (≤ 10 s), pitch/roll은 > 20 s 가능

## 결과 (Sec. 5)

### 정성적
- Pitch (Fig. 9, ~20 s) & Heave (Fig. 10, ~6 s) 모두 **위상이 잘 일치**, 진폭은 다소 오차 (컨트롤러 게인으로 흡수 가능)
- 별도 비행 시퀀스 cross-validation (Fig. 11/12) — 진폭 과소추정이지만 위상 양호

### Table 1 — 식별된 파라미터 (Appendix A.3)
| 파라미터 | 설명 | 식별방법 | 값 | 단위 |
|---|---|---|---|---|
| m | 질량 | Measured | 0.095 | kg |
| Ixx | 관성 (x) | CAD/PEM | 1.24e-4 | kg·m² |
| Iyy | 관성 (y) | CAD/PEM | 1.30e-4 | kg·m² |
| Izz | 관성 (z) | CAD/PEM | 6.66e-5 | kg·m² |
| z_dw | CoG→하부 hub 거리 | CAD | −0.051 | m |
| z_up | CoG→상부 hub 거리 | Measured | −0.091 | m |
| Θ_SPmax | swash plate max angle | Measured | 15 | ° |
| R | 로터 반경 | Measured | 0.0875 | m |
| c_Tdw | 하부 추력계수 | Test bench | 0.0115 | – |
| c_Tup | 상부 추력계수 | Test bench | 0.0131 | – |
| c_Qdw | 하부 토크계수 | Test bench | 0.0018 | – |
| c_Qup,rot | 상부 로터 토크계수 | Test bench | 0.0019 | – |
| c_Qup,stab | stabilizer bar 토크계수 | Hover/PEM | 3.58e-5 | – |
| J_drive,dw | 구동계 관성 (하부) | CAD/PEM | 1.914e-5 | kg·m² |
| J_drive,up | 구동계 관성 (상부) | CAD/PEM | 9.78e-6 | kg·m² |
| κ_E | 전기 모터상수 | LS | 0.0045 | V·s⁻¹ |
| κ_M | 기계 모터상수 | LS | 0.0035 | Nm/A |
| d_R | 모터 마찰 | LS | 5.2107e-7 | Nm·s |
| R_Ω | armature 저항 | LS | 1.3811 | Ω |
| i_gear | 기어비 | Measured | 1.5 | – |
| η_gear | 기어 효율 | Measured/PEM | 0.84 | – |
| W_hub | fuselage drag | Hover/PEM | 0.0108 | N |
| T_f,dw | swash+rotor 시정수 (하부) | PEM | 0.001 | s |
| T_f,up | stabilizer bar 시정수 (상부) | PEM | 0.16 | s |
| l_dw | linkage factor 하부(swash) | PEM | 0.41 | – |
| l_up | linkage factor 상부(stab) | PEM | 0.83 | – |

> 주목 포인트: `T_f,up = 0.16 s`로 비교적 길다 → stabilizer bar의 강한 저역통과 효과가 attitude 응답 형상을 지배. `T_f,dw = 0.001 s`로 매우 짧음 → 하부 swash 응답은 사실상 즉답.

## COAX-SIM 연관

### 1) `Dynamics/RotorDynSchafroth.m`
- 모터+로터 결합식을 단순화하여 구현:
```
J_rot·Ω̇ = K_motor·(Ω_cmd − Ω) − γ·Ω² + γ·Ω_cmd²
```
- **유도 출처:** 본 논문 Eq. (18) (BLDC + drag torque)
  - `(κ_M·U_bat·u_mot,i − κ_M·κ_E·i_gear·Ω_i)/(i_gear·R_Ω) − d_R·Ω_i` 부분이 COAX-SIM에서 `K_motor·(Ω_cmd − Ω)` 형태로 1차 lag로 흡수됨 (정상상태 `Ω_ss = Ω_cmd` 조건).
  - 두 번째 `c_Qi·k_Q·Ω²/(i_gear²·η_gear)` aerodynamic drag torque → COAX-SIM의 `γ·Ω²` 항.
  - COAX-SIM이 `+γ·Ω_cmd²` 항을 추가한 이유는 steady-state에서 `Ω_ss = Ω_cmd` 평형이 성립하도록 인위적 보정 (논문 원식은 비선형이라 평형 RPM이 `Ω_cmd`와 별개).
- 즉 COAX-SIM의 `K_motor ↔ κ_M·κ_E/R_Ω` 등 모터상수 묶음, `γ ↔ c_Q·k_Q/(i_gear²·η_gear)` 로 대응.

### 2) `Dynamics/CoaX_Dyn.m`
- 본 논문의 **Eq. (8), (9), (10), (33)–(36)** 을 직접 구현:
```matlab
T_up = c_T(1) * pi * rho * D^4 * Omega_up^2;    % Eq.(8) — 상부 추력
T_dw = c_T(2) * pi * rho * D^4 * Omega_dw^2;    % Eq.(8) — 하부 추력
Q_up = c_Q(1) * pi * rho * D^5 * Omega_up^2;    % Eq.(9) — 상부 토크
Q_dw = c_Q(2) * pi * rho * D^5 * Omega_dw^2;    % Eq.(9) — 하부 토크
```
- `D = 2R` 로 `D^4 = 16·R^4` 이므로 `c_T(논문) = c_T(COAX-SIM)·16`의 스케일 차이가 있음 → COAX-SIM 사용 시 c_T 값을 16으로 나눠 본 논문 값과 맞춰야 함.
- 하부 로터 추력은 `eul2rotm([0, β, α]) * [0;0;-T_dw]` 로 TPP tilt 적용 = 본 논문 `n_Tdw = [cosα·sinβ; sinα; −cosα·cosβ]`와 등가.
- **상부 로터는 tilt 없이 z축 추력만** 사용 (`Thrust_up = [0;0;-T_up]`) — 본 논문에서는 stabilizer bar로 `(α_up, β_up)` 발생. **COAX-SIM은 단순화를 위해 stabilizer bar 동역학을 생략한 변형판**.
- 모멘트:
```matlab
Moment(1) = r_cd * T_dw * sin(alpha);              % ↔ 본 논문 M_x 의 하부 기여
Moment(2) = r_cd * T_dw * cos(alpha) * sin(beta);  % ↔ 본 논문 M_y 의 하부 기여
Moment(3) = Q_up - Q_dw;                            % ↔ 본 논문 M_z (c_Qup,stab 무시)
```
- `r_cd ↔ |z_dw|` (CoG↔하부 hub 거리)에 대응.
- COAX-SIM은 **상부 hub arm × 상부 추력 항이 빠져있음** (`z_up·T_up·sinβ_up` 등) → stabilizer bar 미포함 단순화의 결과.

### 3) COAX-SIM에 그대로 적용 가능한 정량값
- `m = 0.095 kg`
- `I = diag(1.24e-4, 1.30e-4, 6.66e-5) kg·m²`
- `R = 0.0875 m` → `D = 0.175 m`
- `c_T` (CoaX_Dyn 정의에 맞춰 변환 후): `c_T,up = 0.0131/16 ≈ 8.19e-4`, `c_T,dw = 0.0115/16 ≈ 7.19e-4`
- `c_Q` (마찬가지): `c_Q,up = 0.0019/16`, `c_Q,dw = 0.0018/16`
- `z_dw = -0.051 m` (`r_cd ≈ 0.051 m`)
- 모터 묶음: `K_motor`, `γ`는 본 논문의 `κ_M`, `κ_E`, `R_Ω`, `d_R`, `c_Q`, `i_gear`, `η_gear`로 환산
- `T_f,up = 0.16 s`, `T_f,dw = 0.001 s` (stabilizer/swash 시정수)
- `θ_SPmax = 15°`, `l_dw = 0.41`, `l_up = 0.83`
- `W_hub = 0.0108 N` (fuselage downwash drag)

## 핵심 수식 모음 (재현용)

```
(2)  A_BJ  : NED→Body Euler 변환행렬 (cφsψ 등 9성분)
(5)  [u̇;v̇;ẇ] = F/m − ω × v
(6)  [ṗ;q̇;ṙ] = I⁻¹(M − ω × Iω)
(8)  T_i = c_Ti · π·ρ·R⁴·Ω_i²
(9)  Q_i = c_Qi · π·ρ·R⁵·Ω_i²
(10) n_Ti = [cosα·sinβ; sinα; −cosα·cosβ]    (TPP tilt 방향)
(14) η̇_bar = (φ − η_bar)/T_f,up,  ζ̇_bar = (θ − ζ_bar)/T_f,up
(15) α_up = l_up·(η_bar − φ),     β_up = l_up·(ζ_bar − θ)
(16) α̇_dw = (−l_dw·u_serv2·θ_SPmax − α_dw)/T_f,dw  (β_dw 동일)
(17) J_mot·ω̇ = (κ_M·U − κ_M·κ_E·ω)/R_Ω − d_R·ω − M_L
(18) J_drive·Ω̇_i = (κ_M·U_bat·u_mot − κ_M·κ_E·i_gear·Ω)/(i_gear·R_Ω)
                  − d_R·Ω − c_Qi·k_Q·Ω²/(i_gear²·η_gear)
(20) Hover : T_up + T_dw = G + W_hub,  Q_up,rot + Q_up,stab = Q_dw
(23) Θ_min = argmin det( R_N(Θ) )          (PEM cost)
(24) A_pitch, B_pitch (선형화 pitch subsystem)
(25) A_heave, B_heave (선형화 heave subsystem)
(35) F_x,F_y,F_z 닫힌 형태
(36) M_x,M_y,M_z 닫힌 형태
```

## CEP 논문(Schafroth 2010, CEP)과의 차이

| 구분 | JIRS 2010 (본 논문) | CEP 2010 |
|---|---|---|
| **주된 초점** | 모델링·식별 절차 | 모델링·식별·**robust 제어 합성**까지 포함 |
| 모델 유도 상세 | 매우 자세 (식 27–44 부록 포함) | 모델은 요약, 제어 비중 ↑ |
| 식별 방법 | PEM + LS + 호버 매칭 | 동일 PEM 사용 (자매 논문) |
| 제어기 설계 | **없음** (서론에 향후 LQG/H∞ 언급만) | H∞ 강인제어 합성 및 비행시험 결과 |
| Stabilizer bar 동역학 | 1차 lag, `T_f,up=0.16 s`, `l_up=0.83` | 동일 모델 활용 |
| 18-state 비선형 모델 | 본문/부록 모두 명시 | 차원만 언급, 상세 유도는 본 논문 참조 |
| x/y 동역학 | 식별 불가 (센서 부재), future work | 동일 한계 |

**요약**: 본 논문은 muFly **물리 모델 + 식별 방법론 전용 reference**. 강인제어(H∞) 설계가 필요하면 CEP 자매 논문을 함께 참조. COAX-SIM의 `RotorDynSchafroth.m`/`CoaX_Dyn.m`은 본 논문 Eq. (8), (9), (10), (18), (33)–(36)을 단순화한 구현이며, **stabilizer bar 동역학(Eq. 14–15)이 빠져있어** 상부 로터를 z축 고정 추력으로 처리함. 향후 stabilizer bar 모델 추가 시 본 논문의 Eq. (14)–(15) 와 `T_f,up=0.16 s, l_up=0.83`을 그대로 도입 가능.
