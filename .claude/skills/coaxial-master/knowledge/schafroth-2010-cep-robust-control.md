# Schafroth 2010 (CEP) — muFly 동축반전 마이크로헬리콥터 강인 제어

## 메타데이터
- **저자:** D. Schafroth, C. Bermes, S. Bouabdallah, R. Siegwart
- **소속:** Autonomous Systems Lab, Tannenstr. 3, ETH Zurich, 8092 Zurich, Switzerland
- **저널:** Control Engineering Practice, Vol. 18, pp. 700–711
- **발행년도:** 2010 (Received 2 April 2009 / Accepted 2 February 2010 / Available online 16 February 2010)
- **DOI:** 10.1016/j.conengprac.2010.02.004
- **시스템:** muFly micro coaxial helicopter (mass 95 g, span 17 cm, height 15 cm)
- **프로젝트:** muFly (European 6th Framework Programme STREP, FP6-2005-IST-5-call 2.5.2 Micro/Nano Based Sub-Systems, Contract no. FP6-IST-034120)

## 핵심 기여
- **Modeling:** muFly 동축반전 마이크로헬리콥터의 완전한 비선형 6DOF 동역학 모델을 강체운동 기반으로 구축. stabilizer bar(상부 로터), swash plate(하부 로터), brushless DC(BLDC) 모터, gyroscopic torque, fuselage drag 등 모든 주요 효과 포함. 총 18 states, 4 inputs.
- **System identification:** Covariance Matrix Adaptation Evolution Strategy (CMA-ES) 라는 비선형 randomized search 알고리즘을 사용하여 실비행 데이터에서 결합·비분리 파라미터들을 동시 식별. 시스템을 heave-yaw(14 states 중 14)와 pitch-roll 두 subsystem으로 분리하여 식별.
- **Robust control:** 식별된 비선형 모델을 호버 점에서 선형화한 뒤, dual 2-dof GS/T-weighting scheme 의 mixed sensitivity H∞ 제어기 두 개(heave-yaw, roll-pitch) 설계. 시뮬레이션과 실비행 모두에서 성공적으로 검증.

## 시스템 개요
### muFly 하드웨어
- **질량:** 95 g
- **크기:** 17 cm span × 15 cm height
- **로터 직경:** R = 0.0875 m (Table 1)
- **로터 구동:** 2개의 lightweight brushless DC (BLDC) 모터 (counter rotating, aerodynamic drag 토크 상쇄)
  - Yaw: 두 로터의 differential rotor speed 로 제어
  - Altitude: 두 로터의 simultaneous rotor speed 변화로 제어
- **기계적 구조:**
  - **Stabilizer bar:** 상부 로터에 장착, 자세 안정화용 (RC 모델 헬리콥터 방식). 높은 inertia로 동체의 roll/pitch 움직임에 lag → cyclic pitch 입력으로 redress moment 생성
  - **Swash plate:** 하부 로터에 장착, 2개 servo로 작동, 자세(roll/pitch)를 직접 조향
  - Battery: lithium polymer
- **센서:**
  - IMU (자세 측정)
  - Ultrasonic distance sensor (지면거리 측정)
  - dsPIC microprocessor + Bluetooth (지상국 통신)
- **신호:** 모든 motor controller / servo 입력은 PPM (pulse position modulated) 신호

### 좌표계 정의
- **Inertial frame J** + **body-fixed frame B** (CoG 기준 body frame)
- 본체 속도: `[u, v, w]`, 각속도: `[p, q, r]`, 자세: `[φ, θ, ψ]`, 위치(NED 표시): `[N, E, D]` (Fig. 5 참조)
- 부호 규약: lower rotor는 위에서 봤을 때 시계방향(CW), upper rotor는 반시계방향(CCW). Thrust는 rotor axis(z-axis) 방향, 본 논문에서 thrust vector `t_i = T_i · n_T,i`.

## 동역학 모델
### 강체 6DOF (논문 Section 3 식 (1), (2))
Newtonian mechanics 기반, CoG-기반 body frame에서의 운동방정식:

```
[u̇, v̇, ẇ]ᵀ = (1/m) f − [p, q, r]ᵀ × [u, v, w]ᵀ
[ṗ, q̇, ṙ]ᵀ = I⁻¹ ( m − [p, q, r]ᵀ × I[p, q, r]ᵀ )
```

여기서 `m` = total mass, `I` = body inertia tensor, `f` = total external force vector, `m` = total external moment vector.

### 외력 / 모멘트 합 (Section 3)
```
f = t_up + t_dw + g + w_hub
m = q_up + q_dw + r_Cup × t_up + r_Cdw × t_dw + q_gyro,dw + q_gyro,up
```

- `t_up, t_dw` : 상/하 로터 thrust vector
- `g` : 중력
- `w_hub` : aerodynamic fuselage drag
- `q_up, q_dw` : 로터 drag torque
- `r_Cup × t_up, r_Cdw × t_dw` : thrust가 CoG와 정렬되지 않아 생기는 cross-product moment
- `q_gyro,dw, q_gyro,up` : 로터 가속에 의한 gyroscopic torque (특히 stabilizer bar가 큰 inertia 때문에 상부 로터 가속 시 상당함)

호버 근방 가정 → 동체 병진(translation)에 의한 공력 효과는 무시.

### 로터/액추에이터 모델

#### Rotor thrust & torque (호버, Bramwell 2001)
```
T_i = c_T,i · π · ρ · R⁴ · Ω_i² = c_T,i · k_T · Ω_i²
Q_i = c_Q,i · π · ρ · R⁵ · Ω_i² = c_Q,i · k_Q · Ω_i²
```
- `ρ` : 공기 밀도
- `c_T, c_Q` : thrust / torque coefficient (상/하 로터 다름; 하부는 상부의 downwash 영향으로 따로 식별)
- `Ω_i` : 로터 각속도

#### Thrust vector 방향 (tilt angles α, β)
Tilt angle α (x-axis 회전), β (y-axis 회전)로 정의 (Fig. 2):
```
n_Ti = [ cos(α_i) · sin(β_i)
         sin(α_i)
        −cos(α_i) · cos(β_i) ]
```
Rotor torque vector는 rotor axis (z-axis) 만 가정.

#### Gyroscopic torque
```
Q_gyro,i = J_drive,i · Ω̇_i
```
(rotor axis 방향, 상부의 stabilizer bar inertia 때문에 큰 효과)

#### Stabilizer bar (1차계, Mettler 2003)
상부 로터의 tip path plane (TPP) 동역학을 1차 lag로 모델링:
```
η̇_bar = (1/T_f,up) · (φ − η_bar)
ζ̇_bar = (1/T_f,up) · (θ − ζ_bar)
```
- `η_bar, ζ_bar` : stabilizer bar plane 각도
- `T_f,up` : 상부 로터의 following time constant
- 상부 로터의 tilt angle:
```
α_up = l_up · (φ − η_bar)
β_up = l_up · (θ − ζ_bar)
```
- `l_up` : linkage factor (upper rotor)

Fig. 4: 초기 roll 20° 외란에 대한 stabilizer bar 모델의 redress 응답 → 짧은 시간에 호버로 복귀 확인.

#### Swash plate (1차계, 하부 로터)
서보 입력 → TPP 각도 변화도 1차계로 모델링. 서보/로터 동역학 모두 시간상수 `T_f,dw` 로 묶어 표현:
```
α̇_dw = (1/T_f,dw) · ( −l_dw · u_serv2 · θ_SPmax − α_dw )
β̇_dw = (1/T_f,dw) · ( −l_dw · u_serv1 · θ_SPmax − β_dw )
```
- `θ_SPmax` : 최대 swash plate tilt angle
- `l_dw` : linkage factor (lower rotor)
- `u_serv,i` : 서보 입력 (스케일링 `u_serv ∈ {−1, 1}`)

#### BLDC 모터 모델 (Mueller & Ponick 2006, 기어 확장)
BLDC가 speed control / speed measurement 출력이 없어 전기모터 자체를 모델링:
```
J_drive · Ω̇_i = ( κ_M · U_bat · u_mot,i − κ_M · κ_E · i_gear · Ω_i ) / ( i_gear · R_Ω )
                − d_R · Ω_i − ( c_Q,i · k_Q · Ω_i² ) / ( i_gear² · η_gear )
```
- `J_mot` : moment of inertia
- `κ_E, κ_M` : 전기/기계 motor constant
- `R_Ω` : 전기저항
- `d_R` : gear ratio 마찰계수
- `i_gear, η_gear` : gear ratio / efficiency
- `U_bat` : battery voltage
- `u_mot,i` : motor PPM 입력 (스케일링 `u_mot ∈ {0, 1}`)

### State / Input 벡터
**States (18개):**
```
x = [x, y, z, u, v, w, φ, θ, ψ, p, q, r, α_dw, β_dw, α_up, β_up, Ω_dw, Ω_up]ᵀ
```

**Inputs (4개):**
```
u = [u_mot,dw, u_mot,up, u_serv1, u_serv2]ᵀ
```

### 파라미터 표 (Table 1, Appendix A — 식별된 정량값)
| Parameter      | Description                          | Value      | Unit          |
| -------------- | ------------------------------------ | ---------- | ------------- |
| m              | Mass                                 | 0.095      | kg            |
| I_xx           | Inertia around x-axis                | 1.12e-4    | kg m²         |
| I_yy           | Inertia around y-axis                | 1.43e-4    | kg m²         |
| I_zz           | Inertia around z-axis                | 2.66e-5    | kg m²         |
| z_dw           | Distance CoG-lower rotor hub         | −0.051     | m             |
| z_up           | Distance CoG-upper rotor hub         | −0.091     | m             |
| Θ_SP,max       | Maximal swash plate angle            | 15         | deg           |
| R              | Rotor radius                         | 0.0875     | m             |
| c_T,dw         | Thrust coefficient lower rotor       | 0.0117     | –             |
| c_T,up         | Thrust coefficient upper rotor       | 0.0138     | –             |
| c_Q,dw         | Torque coefficient lower rotor       | 0.0018     | –             |
| c_Q,up         | Torque coefficient upper rotor       | 0.0025     | –             |
| J_drive,dw     | Drive train inertia (down)           | 7.04e-6    | kg m²         |
| J_drive,up     | Drive train inertia (up)             | 2.11e-5    | kg m²         |
| κ_E            | Electrical motor constant            | 0.0045     | V⁻¹ s⁻¹       |
| κ_M            | Mechanical motor constant            | 0.0035     | Nm A⁻¹        |
| d_R            | Motor friction                       | 5.2107e-7  | Nm s          |
| R_Ω            | Resistance                           | 1.3811     | Ω             |
| i_gear         | Gear ratio                           | 1.5        | –             |
| η_gear         | Gear efficiency                      | 0.84       | –             |
| W_hub          | Drag force on the fuselage           | 0.009      | N             |
| T_f,dw         | Following time lower rotor           | 0.001      | s             |
| T_f,up         | Following time upper rotor           | 0.24       | s             |
| l_dw           | Linkage factor lower rotor           | 0.77       | –             |
| l_up           | Linkage factor upper rotor           | 0.48       | –             |

> 주: 하부 c_T(0.0117)가 상부 c_T(0.0138)보다 작은 것은 상부의 downwash 영향. 상부 T_f,up=0.24s가 하부 T_f,dw=0.001s보다 훨씬 크다 → stabilizer bar의 느린 동역학.

## 시스템 식별

### 사전 측정 파라미터
- **CAD 도출:** mass, max swash plate angle, gear ratio, rotor diameter, body inertia (전부 mechanical 특성)
- **테스트 벤치:**
  - **공력 계수 `c_T, c_Q`** : coaxial rotor test bench (Schafroth, Bouabdallah, Bermes, & Siegwart, 2008)에서 식별. 동축 구성으로 운영하여 상부의 downwash가 하부 로터에 미치는 영향 포함
  - **BLDC motor 파라미터** : 파트너인 CEDRAT(2009)에서 정전압 인가 + 외부 토크 변화 + 회전속도/전류 측정. Motor equation의 `ω̇ = 0` (정상상태) 해를 least-square (LS, Schwarz & Kaeckler 2004)로 식별 → Fig. 6
- **남은 결합 파라미터:** 정적 식별 불가, 동적(실비행) 데이터로 CMA-ES 식별

### CMA-ES 식별 절차
- **CMA-ES (Hansen & Ostermeier 1996):** randomized 2차 search algorithm. 양정치 covariance matrix `C`를 quasi-Newton의 inverse Hessian 대신 추정. ill-conditioned, non-separable, non-linear, non-convex, 3–100차원에 강인. 사전 파라미터 경계 설정이 중요.
- **비행 데이터 수집:** 파일럿이 조종, 가능한 모든 주파수 대역을 커버하기 위해 pilot 입력에 chirp signal 중첩. open-loop 조종 불가 → PID 추가, PID 입력(파일럿 reference)을 식별 입력으로 사용.
- **식별 블록도 (Fig. 7):**
  - `r(t)` (pilot reference) → PID → `u(t)` → muFly 실기 → `y(t)` (실측 IMU/ultrasonic)
  - 동일한 `u(t)`를 비선형 모델에 → 예측 `ŷ(t)`
  - `ε = y − ŷ` 를 CMA-ES로 `Θ`(파라미터) 조정해 최소화
- **서브시스템 분리:** sensor (IMU + ultrasonic) 한정 → 14 states로 축소.
  - **Heave-yaw** : altitude 고정·yaw 가진(dataset 1) + altitude 가진·yaw 고정(dataset 2) 두 데이터로 분리 식별
  - **Pitch-roll** : gyroscopic effect 결합으로 동시 식별

### 식별 결과
- 비선형 모델과 실험 데이터가 well-matched (Figs. 8–11). 진폭에서 약간 차이가 있으나 closed-loop에서는 controller gain으로 흡수 가능.
- Fig. 8 : heave 액추에이션 → 모터 PPM, altitude, yaw rate 비교
- Fig. 9 : yaw 액추에이션 → 동일
- Fig. 10 : roll 액추에이션 → 서보 PPM, roll angle 비교
- Fig. 11 : pitch 액추에이션 → 서보 PPM, pitch angle 비교

## 강인 제어기 설계

### 전체 제어 구조 (Fig. 12)
3개의 MIMO 컨트롤러 계층 구조:
1. **Position controller** → 위치 `x, y, z` → roll/pitch reference 생성
2. **Heave-yaw controller** → `z, ψ` ↔ `u_mot,dw, u_mot,up` (두 로터 속도)
3. **Roll-pitch controller** → `φ, θ` ↔ `u_serv1, u_serv2` (swash plate)

> 본 논문에서는 position sensor 미완성으로 position controller 제외. heave-yaw 와 roll-pitch H∞ controller만 설계.

### 미션 가정
- 빠른 기동 불필요 → 호버 근방 정밀 제어가 핵심 → **선형 제어기**가 합리적 → **mixed sensitivity H∞ optimization** 선택

### Plant 선형화
비선형 모델을 호버 점에서 linearization → 두 subsystem:
- **Heave-yaw plant** : 6 states, state-space
- **Roll-pitch plant** : 8 states, state-space

### Weighting scheme: dual 2-dof GS/T (Geering 2004)
- 출력 `y` → 입력 `u` 의 GS/T-weighting. plant `G_s` 의 sensitivity `S` 와 complementary sensitivity `T` 를 가중 `W_i`로 형성.
- **Roll-pitch:** pure feedback controller (1 자유도)
- **Heave-yaw:** 2-dof (Horowitz 1963):
  - `K_f` : feedforward (reference 추종) — altitude/heading 정밀 추종, 오버슈트 최소화
  - `K_b` : feedback (외란 제어)
  - 단점: 마이크로컨트롤러 연산 부담 ↑

### Heave-yaw controller (Section 5.1)
**Transfer function `T_zw` (4×4):**
```
T_zw = [ −W_ū·T_u·W_T̄        G_s·S_u·W_T̄
         W_d·S_u·W_r·K_f      T_yr·W_r̄
         W_d·S_u·K_b·W_r      (T_yr − W_M)·W_r
         W_d·S_u·W_d           T_e·W_d         ]
```
H∞ 문제: `‖T_zw‖_∞ ≤ γ`, `γ ≤ 1` 이면 사양 충족. 실용적으로 `γ = 1` 도달은 불필요, 작은 `γ` 면 asymptotically stable & robust (Geering 2004).

**식별된 plant (Fig. 15):** Heave-yaw plant `G_s` 의 cross-over frequency = 2.82 rad/s, integrating character. → controller에 순수 integrator 대신 imaginary axis 근방 pole 배치 (battery voltage drop 보상용).

**Weighting 함수 (논문 명시 값):**
```
W_ū(s) = 0.001                               (2번째 row 영향 최소화, 무시 가능)
W_d(s) = (s + 10) / (0.1 s + 20)             (complementary sensitivity T_e의 경계)
W_T̄(s) = (0.7 s + 1) / (s + 0.7/100)
W_r̄(s) = (s + 7) / (0.01 s + 14)
W_r(s) = (s² + 15 s + 14) / (10⁻⁴ s² + 14 s + 0.0014)   (sensitivity S_u 형성)
W_M(s) = 14 / (s + 14)                       (reference model)
```

**결과:**
- `γ = 1.38`
- Cross-over frequency `ω_c = 3.26 rad/s`
- Maximum sensitivity peak `S_u,max = 3.7398 dB`
- Maximum complementary sensitivity peak `T_u,max = 3.93 dB`
- Step response (heave, −0.5 m, Fig. 17): 비선형 모델 시뮬에서 오버슈트 없이 reference 추종. Yaw step도 동일.
- 실비행 (Fig. 18): 시뮬보다 약간 떨어지는 성능 (모델 편차, sensor noise), yaw는 IMU drift로 인해 시뮬 예측보다 느슨.
- **Robustness test (Fig. 19):** `c_T,up`, `c_Q,up`, `J_drive,up` 을 nominal에서 ±20% 변동 → controller가 강인하게 동작 (초기 implied operating point difference로 약간의 성능 저하 후 회복). steady-state error 약간 존재 (pole이 imaginary axis 근방, 순수 적분기 아님).

### Roll-pitch controller (Section 5.2)
- Heave-yaw와 비슷한 절차이나 weighting scheme 만 다름 (feedforward `K_f` 제거).
- Roll-pitch plant 특성 (Fig. 20):
  - **매우 좁은 대역폭** (stabilizer bar 때문)
  - **공진 peak at 14.5 rad/s** (stabilizer bar 동역학)
  - → controller bandwidth 를 **2 rad/s** 로 낮게 설계해 공진을 cut-off 이후로 밀어냄
- Roll-pitch controller의 주 임무: stabilizer bar 의 stabilization 보강 + position controller가 주는 moderate reference 추종
- 시뮬 결과(Fig. 22)와 실비행(Fig. 23): 실비행이 시뮬보다 약간 더 빠른 응답 → 비선형 모델이 pitch 동역학을 underestimate. **stabilizer bar 제거 + 능동 안정화** 시 control authority 개선 가능 (저자 향후 과제).

## 시뮬레이션/실험 결과

### 정량 결과 요약
| 항목                         | 값                                             |
| ---------------------------- | ---------------------------------------------- |
| Heave-yaw γ                  | 1.38                                           |
| Heave-yaw cross-over ω_c     | 3.26 rad/s                                     |
| Heave-yaw S_u,max            | 3.7398 dB                                      |
| Heave-yaw T_u,max            | 3.93 dB                                        |
| Heave-yaw plant ω_c          | 2.82 rad/s (Fig. 15)                           |
| Roll-pitch controller BW     | 2 rad/s (설계 선택)                            |
| Roll-pitch 공진              | 14.5 rad/s (stabilizer bar)                    |
| Heave step                   | −0.5 m, 오버슈트 없음 (sim), 약간의 노이즈 (real) |
| Robustness (±20% 변동)       | c_T,up, c_Q,up, J_drive,up — 안정 유지         |

### 시나리오
- Heave step (−0.5 m, 호버에서 위로 0.5 m)
- Yaw step
- Pitch reference following (pilot RC 입력)

### 검증 데이터
- 시뮬 + 실비행 (real system) 모두 수행, IMU + ultrasonic 측정. 진폭 차이 일부 존재하나 closed-loop에서 흡수.

## COAX-SIM 연관

### 1) 사용 중인 동역학 표현 비교

COAX-SIM 의 `C:\Users\jsp99\Work\COAX-SIM\Dynamics\CoaX_Dyn.m`:
```matlab
T_up = p.c_T(1) * pi * p.rho * p.D^4 * Omega_up^2;
T_dw = p.c_T(2) * pi * p.rho * p.D^4 * Omega_dw^2;
Q_up = p.c_Q(1) * pi * p.rho * p.D^5 * Omega_up^2;
Q_dw = p.c_Q(2) * pi * p.rho * p.D^5 * Omega_dw^2;
```
→ **본 논문의 식 `T_i = c_T,i · π · ρ · R⁴ · Ω_i²` 와 정확히 동일 형태** (R 대신 D=2R 사용으로 c_T 스케일 조정). Schafroth 모델을 그대로 차용.

COAX-SIM:
```matlab
Thrust_dw = eul2rotm([0, beta, alpha]) * [0; 0; -T_dw];   % tilt 적용 (하부만)
Moment(1,:) = p.r_cd * T_dw * sin(alpha);                 % roll moment from lower rotor tilt
Moment(2,:) = p.r_cd * T_dw * cos(alpha) * sin(beta);     % pitch moment from lower rotor tilt
Moment(3,:) = Q_up - Q_dw;                                % yaw moment from torque diff
```
→ **본 논문의 swash plate (하부 로터) tilt 모델**과 정확히 일치. 상부는 tilt 없이 z-axis thrust만 (논문의 `t_up = T_up · n_T,up`, stabilizer bar 가 없을 때) — COAX-SIM은 stabilizer bar 모델은 제외하고 swash plate만 사용.

### 2) `compute_rotor_speed_CoaX.m` 의 매핑

`C:\Users\jsp99\Work\COAX-SIM\Controller\compute_rotor_speed_CoaX.m`:
```matlab
Omega_up_d = sqrt( (-gamma_2*Thrust_d + k_2*torque_psi_d) / (k_1*gamma_2 + k_2*gamma_1) );
Omega_dw_d = sqrt( (-gamma_1*Thrust_d - k_1*torque_psi_d) / (k_1*gamma_2 + k_2*gamma_1) );
alpha_d   = torque_phi_d   / (k_2 * Omega_dw_d^2 * r_cd);
beta_d    = torque_theta_d / (k_2 * Omega_dw_d^2 * r_cd);
```
→ **본 논문의 동축반전 control allocation의 역해**. `T_i = k_i · Ω_i²`, `Q_i = γ_i · Ω_i²` (논문 식 (4),(5)와 동일 구조)에서 thrust = T_up + T_dw, yaw moment = Q_up − Q_dw 로 두 식을 연립해 `Ω_up, Ω_dw`를 구하고, swash plate tilt `α, β`는 하부 thrust × r_cd × sin/cos 의 역해 (논문 Fig. 3 메커니즘).

### 3) COAX-SIM의 `main.m` 파라미터 정의 위치
`C:\Users\jsp99\Work\COAX-SIM\main.m` 의 38–80번째 줄:
- `p.cT_hover = 0.091659`, `p.cQ_hover = 0.003523` (호버 정의, 표준 컨벤션 `T = c_T · ρ · n² · D⁴`, n = rev/s) → 논문의 `c_T,dw=0.0117, c_T,up=0.0138, c_Q,dw=0.0018, c_Q,up=0.0025` 와는 **컨벤션 단위가 다름** (논문: T = c_T · π · ρ · R⁴ · Ω², COAX-SIM에서 내부 변환 `k_total = cT_hover · ρ · D⁴ / (2π)²`).
- `p.kT_interf = 0.85` (하부 로터 thrust 15% 손실), `p.kQ_interf = 0.90` → **논문의 `c_T,dw/c_T,up = 0.0117/0.0138 ≈ 0.848`, `c_Q,dw/c_Q,up = 0.0018/0.0025 = 0.72`** 와 비교 가능. COAX-SIM의 thrust 비율은 본 논문과 거의 일치 (0.85 vs 0.848), torque 비율은 보수적으로 더 큼 (0.90 vs 0.72).
- `p.rotor_dyn_mode = 3 % Schafroth rotor + servo lag` ← **본 논문이 직접 인용된 모드**. mode 3에서 `p.tau_sw, p.tau_rot, p.J_rot, p.K_motor` 사용 → 논문의 swash plate 1차 lag (`T_f,dw`), stabilizer bar 1차 lag (`T_f,up`), BLDC 모터 inertia/gain 모델에 대응.

### 4) Schafroth 사용 가능 파라미터 (논문 → COAX-SIM 매핑)
| 논문 변수    | 논문 값      | COAX-SIM 변수       | 비고                                |
| ------------ | ------------ | ------------------- | ----------------------------------- |
| T_f,dw       | 0.001 s      | p.tau_sw            | swashplate / 하부 로터 1차 시간상수 |
| T_f,up       | 0.24 s       | (stabilizer bar)    | COAX-SIM은 stabbar 미사용           |
| J_drive,dw   | 7.04e-6      | p.J_rot (×scale)    | 하부 drive train inertia            |
| J_drive,up   | 2.11e-5      | (스케일 환산 필요)  | 상부 drive train inertia            |
| Θ_SP,max     | 15 deg       | p.tilt_max          | 15 deg → COAX-SIM에서도 deg2rad(15) |
| c_T,dw/c_T,up| 0.848        | p.kT_interf=0.85    | thrust interference ratio           |
| c_Q,dw/c_Q,up| 0.72         | p.kQ_interf=0.90    | torque interference ratio (차이 큼) |

> COAX-SIM은 muFly(95g) 대비 훨씬 큰 11.5 kg, D=0.88 m 드론을 대상으로 하므로 절대값(`m, I, R`)은 다르고, **모델 구조와 무차원 비율(interference ratio, 1차 lag 형태)만 차용** 한 형태.

## 핵심 수식 모음 (재현용)

1. **6DOF 강체 동역학:**
```
[u̇, v̇, ẇ]ᵀ = (1/m)·f − [p, q, r]ᵀ × [u, v, w]ᵀ
[ṗ, q̇, ṙ]ᵀ = I⁻¹·( m − [p, q, r]ᵀ × I[p, q, r]ᵀ )
```

2. **Rotor thrust / torque (호버):**
```
T_i = c_T,i · π · ρ · R⁴ · Ω_i²
Q_i = c_Q,i · π · ρ · R⁵ · Ω_i²
```

3. **Thrust vector (tilt 적용):**
```
n_T,i = [ cos(α_i)·sin(β_i), sin(α_i), −cos(α_i)·cos(β_i) ]ᵀ
t_i = T_i · n_T,i
```

4. **Gyroscopic torque (rotor axis 방향):**
```
Q_gyro,i = J_drive,i · Ω̇_i
```

5. **Stabilizer bar 1차 lag:**
```
η̇_bar = (1/T_f,up)·(φ − η_bar)
ζ̇_bar = (1/T_f,up)·(θ − ζ_bar)
α_up   = l_up · (φ − η_bar)
β_up   = l_up · (θ − ζ_bar)
```

6. **Swash plate 1차 lag (서보+로터 합쳐서):**
```
α̇_dw = (1/T_f,dw)·( −l_dw · u_serv2 · θ_SPmax − α_dw )
β̇_dw = (1/T_f,dw)·( −l_dw · u_serv1 · θ_SPmax − β_dw )
```

7. **BLDC 모터 (gear 포함):**
```
J_drive · Ω̇_i = ( κ_M · U_bat · u_mot,i − κ_M · κ_E · i_gear · Ω_i ) / ( i_gear · R_Ω )
                − d_R · Ω_i − ( c_Q,i · k_Q · Ω_i² ) / ( i_gear² · η_gear )
```

8. **외력/모멘트 합:**
```
f = t_up + t_dw + g + w_hub
m = q_up + q_dw + r_Cup × t_up + r_Cdw × t_dw + q_gyro,dw + q_gyro,up
```

9. **H∞ 문제:** `‖T_zw‖_∞ ≤ γ` ( `γ ≤ 1` 이면 사양 충족, 작은 `γ` 면 asymptotically stable & robust ).

10. **Heave-yaw `T_zw` (4 components, dual 2-dof):**
```
T_zw = [ −W_ū·T_u·W_T̄  ;  G_s·S_u·W_T̄  ;
          W_d·S_u·W_r·K_f  ;  T_yr·W_r̄  ;
          W_d·S_u·K_b·W_r  ;  (T_yr − W_M)·W_r  ;
          W_d·S_u·W_d  ;  T_e·W_d  ]
```

## 다른 Schafroth 2010 논문 (JIRS)과의 차이

| 항목       | Schafroth 2010 (JIRS)                  | Schafroth 2010 (CEP) — 본 논문         |
| ---------- | -------------------------------------- | -------------------------------------- |
| 저널       | Journal of Intelligent System (UAV'09 후속) | Control Engineering Practice           |
| 중심 내용  | **모델링 + 시스템 식별** 중심         | **강인 제어 (H∞)** 중심                |
| 모델 상세도| 동일한 비선형 모델 (Schafroth et al. 2009 참조) | 동일한 모델을 **재정리 후 제어 적용**  |
| 식별       | (JIRS에서 다룸)                        | CMA-ES 식별 방법론 + 결과 표 포함      |
| 제어       | 미포함                                 | dual 2-dof GS/T H∞, 두 MIMO controller (heave-yaw + roll-pitch) 설계·검증 |
| 검증       | 식별 검증 (model fit)                  | step response, 실비행, robustness (±20%) |

본 논문(CEP)이 다루는 **추가 내용**:
- H∞ mixed sensitivity 제어기 두 개 (heave-yaw 6-state 2-dof, roll-pitch 8-state 1-dof) 의 명시적 가중함수 (W_ū, W_d, W_T̄, W_r̄, W_r, W_M) 와 γ, ω_c, S_u,max, T_u,max 정량값
- 실비행 검증 (Figs. 18, 23) 및 robustness simulation (±20% 변동 후 회복, Fig. 19)
- Position controller 미구현 이유 (position sensor 미완성) 및 향후 과제 (stabilizer bar 제거, gain scheduling, feedback linearization)

## 향후 과제 (저자 명시)
- Nonlinear control (feedback linearization, gain scheduling) 적용으로 성능 향상
- Stabilizer bar 제거 후 능동 안정화 → drag force 감소, 자율 비행시간 증가, control authority 향상
- Position sensor 완성 후 outer-loop position controller 설계

## 참고: 본 논문이 인용한 핵심 문헌
- Bramwell (2001) — Helicopter dynamics (rotor thrust/torque coefficient 정의)
- Mettler (2003) — Identification modeling of miniature rotorcraft (stabilizer bar 1차 모델)
- Leishman (2006) — Helicopter aerodynamics (TPP)
- Mueller & Ponick (2006) — BLDC motor model
- Geering (2004) — Robuste Regelung (dual 2-dof GS/T weighting)
- Skogestad & Postlethwaite (2005) — Multivariable feedback control (H∞ 이론)
- Hansen & Ostermeier (1996), Hansen (2009) — CMA-ES
- Schafroth, Bouabdallah, Bermes, Siegwart (2008) — Coaxial rotor test bench
- Schafroth, Bermes, Bouabdallah, Siegwart (2009) — muFly 모델 상세 (JIRS / UAV'09)
