# ISAT 2017 — DOB 기반 동축반전 UAV 계층 제어

## 메타데이터
- **저자:** M. Rida Mokhtari (a,b,*), Brahim Cherki (a,b), Amal Choukchou Braham (b)
  - (a) École supérieure des sciences appliquées de Tlemcen, Algeria
  - (b) Laboratoire d'Automatique de Tlemcen (LAT), Electrical Engineering Department, Tlemcen University, Algeria
- **저널:** ISA Transactions 67 (2017) 466–475
- **DOI:** http://dx.doi.org/10.1016/j.isatra.2017.01.020
- **수신/수정/수락:** 2016-05-16 / 2016-11-16 / 2017-01-06 (Available online 2017-01-28)
- **키워드:** Disturbance Observer, Finite time convergence, Hierarchical control, Coaxial-rotor

## 핵심 기여
- **Hierarchical backstepping + Finite-Time Disturbance Observer (FTDO)** 결합한 신규 비선형 제어 알고리즘 제안.
- 비행 제어 문제를 **(i) outer loop (translation/thrust)** 와 **(ii) inner loop (attitude/torque)** 로 분리.
- 각 루프에 독립적인 FTDO를 두어 모델 불확실성·공력 외란을 **유한시간** 내 온라인 추정 및 능동 보상.
- 기존 sliding mode observer 대비 더 빠른 수렴, 강건성, 외란 제거능력 우위 주장.
- 외란 추정값을 명시적으로 안정성 해석(Lyapunov)에 반영하여 폐루프 안정성 보장.

## 시스템 모델 (Coaxial-Rotor UAV)
### 좌표·자세 표현
- 일반화 좌표: `q = [ξ; η]^T`, `ξ = (x,y,z)^T ∈ R^3`, `η = (φ,θ,ψ)^T ∈ R^3` (Euler ZYX).
- 회전행렬 `R_η = R_ψ R_θ R_φ ∈ SO(3)`.
- 자세 운동학 행렬 `Ψ(η)` (3x3), skew-anti `sk(β)` 사용.

### Newton–Euler 동역학 (Eq. 1)
```
m ξ̈ = R_η T  − m g z_e + F_ext
J η̈ = Γ_a − C(η, η̇) η̇ + Γ_ext
```
- `J = J^Ψ(η)`: positive-definite inertia matrix (θ ≠ kπ/2 가정).
- `T = (T_x, T_y, T_z)^T`: 동축반전 두 로터 + cyclic swashplate에 의한 추력 벡터 (Eq. 2).
- `Γ_a = (τ_φ, τ_θ, τ_ψ)^T`: rotor + swashplate 토크 (Eq. 3).
- 원본 4 입력: `(Ω1, Ω2, δ_cx, δ_cy)` — rotor 회전속도 2개 + cyclic swashplate incidence 2개.

### Remark 1 (모델 단순화)
- 횡방향 swashplate 각도 `δ_cx, δ_cy` 가 작다는 가정으로 `cos δ ≈ 1, sin δ ≈ δ`.
- 횡력 `T_x, T_y` 가 수직력 `T_z` 대비 작아 `Σ Γ_a ≈ 0`.
- (2)와 (3)의 변수 변환은 diffeomorphism: `(Ω1, Ω2, δ_cx, δ_cy) ↔ (T_z, Γ_a)`.

### 완전 모델 (Eq. 4)
```
ξ̇ = v
m v̇ = T_z R_η z_e − m g z_e + Σ Γ_a + F_ext
η̇ = w
J ẇ = Γ_a − C(η, w) w + Γ_ext
```
- `Σ = (1/d) sk(z_e)`: small body force coupling 행렬 (실용상 무시).
- **외란 가정:** `F_ext`, `Γ_ext` 는 **알 수 없으나 유계** (wind gusts, internal coupling, unmodelled dynamics 포함).

## 계층 제어 구조 (Fig. 2)
Outer/Inner 두 cascade 서브시스템 + 각 루프 후단 FTDO:

```
[Guidance] → [Translation Ctrl (Eq.31)] → [Nonlinear Transf (Eq.34)] → [Attitude Ctrl (Eq.53)] → [Nonlinear Transf] → Coaxial Rotor
                          ↑                                                 ↑
                    FTDO (Eq.15)                                      FTDO (Eq.47)
```

### Outer loop (위치 / 속도) — Translation Controller
- **목표:** 위치 `ξ` 가 `ξ_d` 추종, 가상 입력 = 원하는 추력 `T_z^d` + 원하는 자세 `R_η^d`.
- **Step 1 (위치 오차):** `δ_1 = ξ − ξ_d`, Lyapunov `L_1 = (1/2) δ_1^T δ_1` (Eq. 6–9).
  - 가상 속도 `ρ_1 = ξ̇_d − K_1 δ_1` (Eq. 8) → `L̇_1 = −K_1 ‖δ_1‖^2 ≤ 0`.
- **Step 2 (속도 오차):** `δ_2 = v − ρ_1` (Eq. 10).
  - `δ̇_2 = (T_z/m) R_η z_e − g z_e − Ψ_1 + F̄_ext` (Eq. 11), `F̄_ext = F_ext/m`.
- **Internal control input (Eq. 31):** `μ_ξ = Ψ_1 − ϑ_1 − δ_1 − K_2 δ_2`
  - 여기서 `ϑ_1` 은 FTDO 출력 (외란 추정값).
- **이상화된 변환 (Eq. 33):** `δ_ξ = μ_ξ − (T_z/m) R_η z_e + g z_e`.

### 명령 변환 (위치명령 → 자세명령, Eq. 34) — 핵심 매핑
주어진 `μ_ξ` 와 `ψ_d` 로부터 추력 크기와 자세각 도출:
```
T_z   = m √(μ_x^2 + μ_y^2 + (μ_z + g)^2)
φ_d   = sin⁻¹[ (m / T_z) · (μ_x sin ψ_d − μ_y cos ψ_d) ]
θ_d   = sin⁻¹[ (m / (T_z cos ψ_d)) · (μ_x cos ψ_d + μ_y sin ψ_d) ]
```
- **`R_η^d` 의 desired attitude 정보가 inner loop reference 가 됨.**

### Inner loop (자세 / 각속도) — Attitude Controller (Eq. 53)
- **Step 3 (자세 오차):** `δ_3 = η − η_d`, `L_3 = (1/2) δ_3^T δ_3` (Eq. 35–40).
  - 가상 각속도 `ρ_3 = η̇_d − K_3 δ_3` (Eq. 39).
- **Step 4 (각속도 오차):** `δ_4 = w − ρ_3` (Eq. 41).
  - `δ̇_4 = J^{-1}[ Γ_a − C(η,w) w + Γ_ext ]` (Eq. 42), 변환 후 `δ̇_4 = Γ̄_a − Ψ_2 + Γ̄_ext` (Eq. 43).
- **Attitude control input (Eq. 53):** `Γ̄_a = Ψ_2 − δ_3 − ϑ_2 − K_4 δ_4`
- **최종 제어 입력 (Eq. 61):**
  ```
  T_z = m ‖ μ_ξ + g z_e ‖ = m ‖ Ψ_1 − ϑ_1 − δ_1 − K_2 δ_2 + g z_e ‖
  Γ_a = J [ Ψ_2 − δ_3 − ϑ_2 − K_4 δ_4 ]
  ```

## DOB 설계 — Finite-Time Disturbance Observer (FTDO)

### Outer loop FTDO (Eq. 12–17, 28)
- 시스템 (11): `δ̇_2 = (T_z/m) R_η z_e − g z_e − Ψ_1 + F̄_ext`.
- **Observer dynamics:**
  ```
  δ̂̇_2 = (T_z/m) R_η z_e − g z_e − Ψ_1 + ϑ_1            (Eq. 12)
  δ̃_2  = δ_2 − δ̂_2                                       (Eq. 13)
  δ̃̇_2 = F̄_ext − ϑ_1                                    (Eq. 14)
  ```
- **유한시간 형식 injection (Eq. 15):**
  ```
  ϑ_1 = λ_0 sig^{σ_1}(δ̃_2) + λ_1 ∫ sig^{σ_2}(δ̃_2(τ)) dτ
  ```
  - `sig^σ(δ̃_2) = |δ̃_2|^σ sign(δ̃_2)`, `σ_1 ∈ ]1/2, 1[`, `σ_2 = 2σ_1 − 1`.
  - 종종 `σ_1 = 1/2` 선택 시 강건성 강화 (Remark 2).
- **수렴 영역 (Eq. 27):** `‖ζ_1‖ ≤ ( h_{1max} ‖q_1‖ / λ_min(Q_1) )^{σ_1/σ_2}`.
- **이득 선정:** `λ_0 > 0`, `λ_1 > h_{2max} ‖q_1‖^2 / σ_1 λ_0^2`.

### Inner loop FTDO (Eq. 44–50)
- 시스템 (43): `δ̇_4 = Γ̄_a − Ψ_2 + Γ̄_ext`.
- 동일 구조:
  ```
  δ̂̇_4 = Γ̄_a − Ψ_2 + ϑ_2                                (Eq. 44)
  ϑ_2  = λ_2 sig^{σ_1}(δ̃_4) + λ_3 ∫ sig^{σ_2}(δ̃_4(τ)) dτ (Eq. 47)
  ```
- 유한시간 내 `δ̃_4 → 0`, `δ̂̇_4 → 0` → `ϑ_2 → Γ̄_ext`.

### 사용된 보조정리 (Lemma 1, Bhat & Bernstein)
- 연속 시스템 `ẋ = f(x)`, `f(0) = 0` 에서 `V̇(x) + γ V^α(x) ≤ 0`, `α ∈ (0,1)` 이면 origin 은 finite-time stable.
- Settling time: `t_R ≤ V^{1−α}(x_0) / (γ(1−α))`.

## 안정성 해석
- **Outer loop closed-loop (Eq. 55):**
  ```
  δ̇_1 = −K_1 δ_1 + δ_2
  δ̇_2 = −K_2 δ_2 − δ_1 + δ_ξ
  ```
  Lyapunov `L_2 = (1/2)(δ_1^T δ_1 + δ_2^T δ_2)` → `L̇_2 = −K_1‖δ_1‖^2 − K_2‖δ_2‖^2 + δ_2^T δ_ξ` (Eq. 57).
- **Inner loop closed-loop (Eq. 58):** `δ̇_3 = −K_3 δ_3 + δ_4`, `δ̇_4 = −K_4 δ_4 + δ_3` → `L̇_4 = −K_3‖δ_3‖^2 − K_4‖δ_4‖^2`.
- **계층 분리 가정:** `δ_ξ` 는 `δ_3 → 0` 일 때 0 (보간 항 `h(η_d, δ_3)` 가 bounded, Eq. 37).
  - Inner loop 가 outer loop 보다 충분히 빠르다는 **time-scale separation** 묵시적 가정.
- **LaSalle's invariance principle** 적용하여 cascade 전체 GAS 결론.

## 시뮬레이션·실험 결과
### 파라미터 (Table 1, CRUAV)
- `g = 9.81 m/s^2`, `m = 0.41 kg`, `d = 0.0676 m`.
- 관성: `J_x = J_y = 1.383e-3`, `J_z = 2.72e-4 kg·m^2`.
- 공력계수: `κ_α = 3.6835e-5`, `κ_β = 3.776e-5 N/rad^2·s^2`, `γ_1 = 1.4765e-6`, `γ_2 = 1.3266e-6 N·m/rad^2·s^2`.

### 제어 이득 (Table 2)
- `K_i, i={1,2,3,4} = [1,1,1]`, `λ_0 = λ_2 = [1,1,1]`, `λ_1 = λ_3 = [0.6,0.6,0.6]`, `σ_1 = 0.6`.

### 시나리오 & 외란 정의
- 관성 불확실성: `ΔJ = 0.5 J kg·m^2`.
- 공력 외력 (10/30/40 s 인가): `F_ext = [sin(0.1t); sin(0.1t); sin(0.1t)]^T` N.
- 공력 토크 (10 s 인가): `Γ_ext = [0.3, 0.3, 0.5]^T · sin(0.1t)` N·m.
- 초기조건: `ξ(0) = [0,0,0.5]^T`, `η(0) = [0,0,0.3]^T`.

### 비교군 결과
- **Case a (DOB 없음):** 외란 인가 시 위치 발산, 추종 실패, 시스템 불안정.
- **Case b (DOB 포함):** 외란에도 trajectory 정밀 추종, 액추에이터 명령 연속·실현 가능.
- **Case c (센싱 노이즈, 0.01 variance 위치속도 + 0.0001 variance 자세속도):** 노이즈 존재 시 액추에이터 신호가 거칠어지나 위치 추종은 유지.
- **Case d (Aggressive path tracking):** DOB 없으면 발산, DOB 있으면 안정 추종 (Fig. 10).
- **Case e (Adaptive backstepping [24]과 비교):** 제안 FTDO 기반이 더 빠른 수렴, 더 정밀한 추종, 더 강건 (Fig. 11).

## COAX-SIM 연관

### 직접 매핑 — 본 논문 vs COAX-SIM 코드
| 논문 요소 | 논문 식 | COAX-SIM 파일 | 비고 |
|---|---|---|---|
| Translation Controller (outer) | Eq. 31, Eq. 33 | `Controller\POS2THR.m` | PD + d_hat_f 보상; `Acc_d = -Kp e - Kd v - d_hat_f / Mass` |
| Position → Attitude 변환 | Eq. 34 (`φ_d, θ_d`) | `Controller\ACC2ATT.m` | `phi_d=asin((u_x sinψ - u_y cosψ)/a_T)` 등 식 형태 거의 동일 |
| Total Thrust 식 | Eq. 34 (`T_z`) | `POS2THR.m` line 46 | `Thrust_d = -‖Acc_d - g z_e‖ · Mass` |
| Attitude Controller (inner, backstepping) | Eq. 53, Eq. 61 (`Γ̄_a`) | `Controller\BACKDOB.m` | `Torque = -J(Z1 - Θ_2d_dd + K2 Z2) - d_hat` (Z1=δ_3, Z2=δ_4 대응) |
| Attitude Controller + 안전제약 | Eq. 53 확장 | `Controller\BACKHOCBFQPDOB.m` | 동일 backstepping + DOB, 추가로 HOCBF-QP 안전제약 |
| Outer FTDO (Eq. 15, ϑ_1) | Eq. 12–15 | `Controller\LDOB_pos.m` | **차이:** COAX-SIM 은 **선형(linear) DOB** (`d_hat/d = L/(s+L)`), 논문은 **유한시간 비선형 DOB** |
| Inner FTDO (Eq. 47, ϑ_2) | Eq. 44–47 | `Controller\LDOB.m` | **차이:** COAX-SIM 은 선형 DOB. 논문 식과 위상 비교 시 sig^σ 항이 빠짐 |
| 외란 모델 | `F_ext, Γ_ext` | `Dynamics\DisGen.m` (추정) | 외력/외토크 인가 메커니즘 |

### 핵심 일치점
- **Cascade 구조 (outer→inner)**: `main.m` 의 호출 순서 `POS2THR → ACC2ATT → BACKDOB / BACKHOCBFQPDOB` 가 Fig. 2 와 일대일 대응.
- **DOB 사용 위치 (양쪽 루프 각각)**: COAX-SIM 의 `LDOB_pos`(translation 외력) + `LDOB`(rotation 외토크) 가 Eq. 15·47 와 같은 자리에 위치.
- **추력·자세 매핑식**: `ACC2ATT.m` 의 두 줄이 Eq. 34 의 `φ_d, θ_d` 식과 형태 동일 (sign 관례 차이 가능).

### 차이점 / 발전 여지
1. **DOB 종류:** COAX-SIM 은 1차 LPF 형태의 **Linear DOB** (`z_dot = -L(d_hat + u)`).
   논문 식 (15)은 `λ_0 sig^σ(δ̃) + λ_1 ∫ sig^σ` 형태 **finite-time nonlinear**.
   → COAX-SIM 에 `NDOB.m` (목록에는 없으나 명시됨) 추가 시 본 논문의 FTDO 를 그대로 구현 가능.
2. **안전제약:** COAX-SIM 의 `BACKHOCBFQPDOB.m` 는 본 논문에 없는 HOCBF-QP 를 추가. 본 논문은 backstepping 만으로 GAS 보장하므로 COAX-SIM 은 그 위에 state/torque 박스 제약을 부가한 확장.
3. **샘플 시간:** 논문은 연속시간 해석, COAX-SIM 은 `p.dt_ctrl` 로 Euler 적분 → 이산화 안정성 별도 검토 필요.

### 본 논문이 COAX-SIM 에 주는 시사점
- COAX-SIM 의 `POS2THR + ACC2ATT + BACKDOB + LDOB_pos + LDOB` 조합은 본 논문 (참조 [1] = Mokhtari 2016 의 동일 그룹) 의 **선형 DOB 변형판** 으로 해석 가능.
- 본 논문의 FTDO 를 도입하면 **유한시간 외란 보상** 으로 transient 성능 개선 기대.
- 안정성 증명 양식 (Lyapunov cascade, Eq. 55–60) 을 그대로 차용 가능.

## 핵심 수식 모음 (재현용)

```
(1)  m ξ̈ = R_η T − m g z_e + F_ext
     J η̈ = Γ_a − C(η, η̇) η̇ + Γ_ext

(4)  ξ̇ = v
     m v̇ = T_z R_η z_e − m g z_e + Σ Γ_a + F_ext
     η̇ = w
     J ẇ = Γ_a − C(η, w) w + Γ_ext

(8)  ρ_1 = ξ̇_d − K_1 δ_1                                  ; virtual position vel
(10) δ_2 = v − ρ_1                                          ; velocity error
(12) δ̂̇_2 = (T_z/m) R_η z_e − g z_e − Ψ_1 + ϑ_1             ; outer DOB
(15) ϑ_1 = λ_0 sig^{σ_1}(δ̃_2) + λ_1 ∫ sig^{σ_2}(δ̃_2) dτ    ; FTDO injection
(31) μ_ξ = Ψ_1 − ϑ_1 − δ_1 − K_2 δ_2                       ; outer ctrl
(34) T_z = m √(μ_x^2 + μ_y^2 + (μ_z + g)^2)
     φ_d = sin⁻¹[ (m/T_z)(μ_x sinψ_d − μ_y cosψ_d) ]
     θ_d = sin⁻¹[ (m/(T_z cosψ_d))(μ_x cosψ_d + μ_y sinψ_d) ]
(39) ρ_3 = η̇_d − K_3 δ_3                                   ; virtual ang vel
(41) δ_4 = w − ρ_3                                          ; ang vel error
(44) δ̂̇_4 = Γ̄_a − Ψ_2 + ϑ_2                                ; inner DOB
(47) ϑ_2 = λ_2 sig^{σ_1}(δ̃_4) + λ_3 ∫ sig^{σ_2}(δ̃_4) dτ
(53) Γ̄_a = Ψ_2 − δ_3 − ϑ_2 − K_4 δ_4                       ; inner ctrl
(61) T_z = m ‖ Ψ_1 − ϑ_1 − δ_1 − K_2 δ_2 + g z_e ‖
     Γ_a = J [ Ψ_2 − δ_3 − ϑ_2 − K_4 δ_4 ]
```

**안정성 부등식**
```
Lemma 1: V̇(x) + γ V^α(x) ≤ 0, α ∈ (0,1) ⇒ finite-time stable
         settling time  t_R ≤ V^{1−α}(x_0) / (γ(1−α))
Outer L̇_2 ≤ −K_1‖δ_1‖^2 − K_2‖δ_2‖^2 + δ_2^T δ_ξ      (Eq. 57)
Inner L̇_4 = −K_3‖δ_3‖^2 − K_4‖δ_4‖^2                  (Eq. 60)
```

## 한계
### 저자가 언급한 한계 (Sec. 5)
- 본 연구는 **quasi-stationary flight** 가정 (소형 swashplate 각도 ≈ 0).
- **Aggressive maneuver** (고속, 큰 자세각) 에 대한 공력 효과는 미반영.
- 미래 과제: 고속·과격 기동 시 공력 모델 정밀화, sensor-less 상태 추정용 nonlinear observer 추가.

### COAX-SIM 적용 시 주의점
1. **Inner/outer bandwidth separation 필요:** 논문은 inner loop 가 outer 보다 충분히 빨라야 cascade 안정성이 보장됨을 묵시적으로 가정. COAX-SIM 의 `K1_pos/K2_pos` vs `K1_bsc/K2_bsc` 이득을 이 원칙에 맞춰 튜닝해야 한다.
2. **DOB cutoff `L_dob` 와 finite-time 이득 `λ_i` 의 호환:** COAX-SIM 은 LDOB, 논문은 FTDO. LDOB 의 `L` 이 너무 크면 노이즈 증폭, 너무 작으면 외란 보상 지연 → 본 논문 Case c (노이즈) 결과 참조.
3. **`ACC2ATT.m` 의 sign 관례:** 본 논문 Eq. 34 와 COAX-SIM 의 `phi_d = asin(1/a_T (u(1)sin ψ − u(2)cos ψ))` 부호 정합성 검증 권장 (NED vs ENU 좌표 차이 가능).
4. **`Σ = (1/d) sk(z_e)` small body force 무시:** 본 논문은 작은 swashplate 가정에서 무시 가능하다고 명시 — COAX-SIM 도 동일하게 처리하고 있는지 확인.
5. **유한시간 vs 점근 안정성:** COAX-SIM 의 LDOB 는 점근 수렴, 본 논문 FTDO 는 유한시간 수렴. **고주파/충격성 외란** 시 차이가 두드러질 수 있음.
6. **HOCBF-QP 와 backstepping 결합 안정성:** 본 논문은 backstepping 단독 안정성 증명만 제공. COAX-SIM 의 `BACKHOCBFQPDOB.m` 의 QP 활성/비활성 전환 시 안정성은 별도 해석 필요 (Molnar-22 등 참고).
