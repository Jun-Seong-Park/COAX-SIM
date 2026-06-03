# Glida 2023 (ISAT) — TDE 기반 모델프리 퍼지 동축반전 드론 궤적 추종

## 메타데이터
- **저자:** Hossam Eddine Glida (LMSE Lab, Univ. of Biskra, Algeria), Abdelghani Chelihi (LI3CUB Lab, Univ. of Biskra), Latifa Abdou (LMSE Lab), Chouki Sentouh (LAMIH-UMR CNRS 8201, Hauts-de-France Polytechnic Univ., France), Gabriele Perozzi (Inria, Univ. of Lille, CRIStAL)
- **저널:** ISA Transactions, Vol. 137, pp. 236–247
- **발행년도:** 2023 (수신 2021-04-06, 수정 2022-12-22, 게재 2022-12-27)
- **DOI:** 10.1016/j.isatra.2022.12.015
- **키워드:** Coaxial rotor drone, Model-free control, Fuzzy logic control, Flower pollination algorithm, Time-Delay Estimation, Trajectory tracking

## 핵심 기여
- 동축반전 드론의 **모델프리(model-free)** 위치·자세 궤적 추종을 위해 **OMFFC(Optimal Model-Free Fuzzy Controller)** 제안. 세 요소의 하이브리드:
  1. **PID** (PD-like) 안정화 항
  2. **TDE(Time-Delay Estimation)** 로 미지의 비선형 동역학·외란 추정
  3. **적응형 퍼지 보상기**로 TDE 추정오차 상쇄
- **FPA(Flower Pollination Algorithm)** 메타휴리스틱으로 PID/TDE 게인 (`Λ_P`, `Λ_I`, `Λ_D`)을 오프라인 최적화 → trial-and-error 한계 극복.
- **Lyapunov 기반 전역점근안정성** 증명 + Barbalat 보조정리로 추종오차 0 수렴 보장.
- 기존 MFC 연구들과 달리 코악시얼 로터에 특화, 6개 부시스템(x, y, z, φ, θ, ψ) 분리형 분산 제어 구조.

## 시스템 모델

### 동축반전 드론 동역학 (6-DOF, underactuated)
- 두 개의 반전 BLDC 모터로 회전하는 로터 + swash-plate incidence angle `δ_cx`, `δ_cy` 를 위한 하부 서보 2개.
- Newton–Euler 정식화 (Eq.2):
  ```
  m·p̈ = R·T - m·g·[0,0,1]^T + F_ext
  J·η̈ = Ψ_η^T·τ - C(η, η̇)·η̇ + τ_ext
  ```
  `T`: 추력 벡터(Eq.4), `τ`: 모멘트 벡터(Eq.5), `C(η,η̇)`: Coriolis 항.
- **Remark 1**: 작은 `δ_cx`, `δ_cy` 가정으로 `T_x`, `T_y` 의 lateral 효과를 `T_z` 대비 무시 → 단순화 가능.
- Earth-frame 모델 (Eq.6):
  ```
  ẍ = (cφsθcψ + sφsψ)(k_a·Ω₁² + k_β·cδ_cx·cδ_cy·Ω₂²)/m
  ÿ = (cφsθsψ - sφcψ)(...)/m
  z̈ = (cφcθ)(...)/m - g
  φ̈ = (I_y-I_z)/I_x · θ̇·ψ̇ - (d·k_β·sδ_cx·Ω₂²)/I_x
  θ̈ = (I_z-I_x)/I_y · φ̇·ψ̇ + (d·k_β·cδ_cx·sδ_cy·Ω₂²)/I_y
  ψ̈ = (I_x-I_y)/I_z · φ̇·θ̇ + (γ₁·Ω₁² - γ₂·Ω₂²)/I_z
  ```

### 부시스템 분해 (Eq.10–11)
6개의 SISO subsystem `i ∈ Ξ = {x, y, z, φ, θ, ψ}` 로 분리:
```
ẋ_{i1} = x_{i2}
ẋ_{i2} = Ψ_i(X, t) + u_i(t)            (Eq.11)
y_i    = x_{i1}
```
여기서 `Ψ_i(X, t) = (1 - b_i⁻¹)·ẋ_{i2} + b_i⁻¹·f_i(X) + b_i⁻¹·h_i(t)` 가 **전체 미지 비선형 함수 + 외란** 을 묶은 항.

### 외란/불확실성 가정
- `Ψ_i(X, t)` 는 **연속·시간에 대해 천천히 변하는** 함수여야 TDE 근사가 유효 (Remark 3).
- `h_i(t)` 는 bounded external disturbance. 시뮬에서 sinusoidal disturbance 추가.

## TDE (Time-Delay Estimation)

### 기본 아이디어
- 직전 샘플(`t - ρ`, `ρ` = 1 sampling period) 의 입력·가속도 측정으로 현재 `Ψ_i(X, t)` 를 근사:
  ```
  Ψ̂_i(X, t) = Ψ_i(X, t - ρ) = u_i(t - ρ) - ẋ_{i2}(t - ρ)         (Eq.18)
  ```
- 즉, **모델 정보 0**, 오직 측정값(input/output) 만으로 미지 동역학 추정.

### TDEC 제어 법칙 (Eq.16, 19)
PD-like 항 + TDE 보상:
```
u_i(t) = ÿ_{id} + Γ_Pi·e_i + Γ_Di·ė_i - Ψ̂_i(X, t)                (Eq.16)
       = ÿ_{id} + Γ_Pi·e_i + Γ_Di·ė_i - u_i(t-ρ) + ẋ_{i2}(t-ρ)    (Eq.19)
```
- `Γ_Pi`, `Γ_Di > 0` 는 pole placement 로 선정.
- TDE 만 쓰면 PD 항 한계 때문에 null steady-state error 불가 → 다음 절 퍼지 보상기 도입.

### TDE 오차 한계
- `Ψ̃_i(X, t) = Ψ_i(X, t) - Ψ̂_i(X, t)` 가 0이 아닌 작은 양으로 잔존.
- Universal approximation 으로 ideal 퍼지: `Ψ̃_i*(X, t) = φ_i*^T·ξ_i(e_i, ė_i) + Δ_i`, 보상 오차 `|Δ_i(t)| < Δ̄_i` (Eq.23–24).

## 퍼지 로직 모듈

### 입력/출력
- **입력:** 추종 오차 `e_i` 와 미분 `ė_i` (각 부시스템마다 독립적인 퍼지 시스템 1개).
- **출력:** TDE 보상항 `τ_fi`.

### 멤버십 함수
- 각 변수당 **3개 fuzzy set** (N: Negative, Z: Zero, P: Positive) → 계산량 최소화.
- 규칙 베이스는 standard "If-Then" 형태, Mamdani 추론, product-inference + weighted average defuzzification (Eq.25).
- Fuzzy basis vector:
  ```
  ξ_{i,h}(e_i) = μ_{ξ_e^h}(e_i)·μ_{ξ_ė^h}(ė_i) / Σ_k μ_{ξ_e^k}·μ_{ξ_ė^k},  h = 1,...,N
  ```

### 적응형 보상 (Eq.27)
```
τ_fi = -φ̂_i^T·ξ_i(e_i, ė_i) - Δ̂_i·sgn(ε_i^T·P_i·B_i)
```
- `sgn(·)` 은 chattering 방지를 위해 시뮬에서 `tanh(·)` 로 대체.
- 적응법칙(Eq.28, 29):
  ```
  φ̂̇_i = ϑ_i·ε_i^T·P_i·B_i·ξ_i(e_i, ė_i)
  Δ̂̇_i = η_i·|ε_i^T·P_i·B_i|
  ```
  `ϑ_i`, `η_i > 0` design parameter, `P_i` 는 Lyapunov 방정식 `P_i·A_i^T + P_i·A_i = -Q_i` 의 해.

### 최적화 대상
**PID/TDE 게인 `{Λ_Pi, Λ_Ii, Λ_Di}` 만 FPA 로 오프라인 튜닝.** 퍼지 멤버십 함수 자체는 고정. (논문 Step 1–4 참조)

## 최적화 절차

### FPA (Flower Pollination Algorithm)
Xin-She Yang 2012 제안 메타휴리스틱.

- **Global pollination (Lévy flight)** (Eq.38):
  ```
  S_j^{t+1} = S_j^t + γ·L(λ)·(S* - S_j^t)
  ```
  `L(λ)` 는 Lévy 분포(Eq.39).
- **Local pollination** (Eq.40):
  ```
  v_j^{t+1} = S_j^t + ε·(S_n^t - S_k^t)
  ```
- Switching probability `℘` 로 global/local 전환 (Eq.41).

### 목적함수
RMSE 기반 (Eq.42):
```
J_i = sqrt(Σ_{ℓ=1}^N (y_{id} - y_i)² / N)
J_P = (J_x + J_y + J_z)/3      (위치)
J_R = (J_φ + J_θ + J_ψ)/3      (자세)
```

### FPA 파라미터 (Table 2)
- Probability switch `℘` = 0.8
- `γ` = 1.5
- Scaling factor `ε` = 0.1
- Iteration = 100
- Search space `[S_L, S_U] = [1, 30]`

### 결과 게인 (Table 3, OMFFC)
- 위치 `{x, y}`: `Λ_Pi = 8.362`, `Λ_Ii = 12.487`, `Λ_Di = 7.651`
- 위치 `z`: `Λ_Pi = 9.349`, `Λ_Ii = 7.364`, `Λ_Di = 10.258`
- 자세 `{φ, θ}`: `Λ_Pi = 5.264`, `Λ_Ii = 3.264`, `Λ_Di = 4.759`
- 자세 `ψ`: `Λ_Pi = 3.694`, `Λ_Ii = 3.115`, `Λ_Di = 3.276`

## 제어기 전체 구조 (Fig. 2)

```
   [Reference y_id] ──→ ⊗ ──e_i──┐
                         ↑       ↓
                        y_i   ┌──────────────────────────────┐
                              │  OMFFC                       │
                              │  ┌────────────────────────┐  │
                              │  │ Λ_Pi·e + Λ_Di·ė        │  │
                              │  │ + Λ_Ii·∫e + ÿ_id   ─→ u_iℓ│
                              │  │  (linear PID-like)     │  │
                              │  └────────────────────────┘  │
                              │  ┌────────────────────────┐  │
                              │  │ FLC (fuzzy compensator)│  │
                              │  │  ─→ τ_fi               │  │
                              │  └────────────────────────┘  │
                              │  ┌────────────────────────┐  │
                              │  │ TDE: -Ψ̂_i(X,t)         │  │
                              │  │  = -u_i(t-ρ)+ẋ_{i2}(t-ρ)│  │
                              │  └────────────────────────┘  │
                              └──────────┬───────────────────┘
                                         │ u_i = u_iℓ + τ_fi - Ψ̂_i
                                         ↓
                                  [Coaxial rotor model]
                                         │
            ┌─── FPA (offline) ────┐    │
            │ Init pop → Cost(RMSE)│←───┘  (objective J_i)
            │ → Global/Local sol   │
            │ → Lévy update        │
            │ → Optimal {Λ_P,I,D}  │
            └──────────────────────┘
```

### 통합 제어 법칙 (Eq.20)
```
u_i(t) = ÿ_{id} + Λ_Pi·e_i + Λ_Di·ė_i + Λ_Ii·∫e_i - Ψ̂_i(X, t) + τ_fi
```

## 안정성 해석

### Theorem (논문 본문)
Eq.20 의 MFFC 법칙 + Eq.18 의 TDE 보상 + Eq.27 의 퍼지 보상기를 사용하면, 적응법칙 (Eq.28, 29)을 만족할 때 추종오차 `e_i` 가 0에 점근수렴하고 폐루프 시스템이 점근안정.

### Lyapunov 함수 (Eq.31, 32)
```
V = Σ V_i
V_i = (1/2)·ε_i^T·P_i·ε_i + (1/(2·ϑ_i))·φ̃_i^T·φ̃_i + (1/(2·η_i))·Δ̃_i²
```

### 핵심 부등식 (Eq.36, 37)
Cauchy-Schwarz 적용 후:
```
V̇ ≤ Σ -(1/2)·ε_i^T·Q_i·ε_i + |ε_i^T·P_i·B_i|·Δ̄_i - Δ̂_i·|ε_i^T·P_i·B_i| - (1/η_i)·Δ̃_i·Δ̂̇_i
   ≤ -Σ (1/2)·ϱ_{i,min}(Q_i)·|ε_i|²
```
→ `V ∈ L_∞`, Barbalat 보조정리로 `e_i, ė_i → 0`.

### TDE error bound
Remark 3: `Ψ_i(x, t)` 가 sampling period `ρ` 에 비해 천천히 변할수록 TDE 오차 ↓. 빠른 외란(고주파)은 TDE 만으로 추적 불가 → 퍼지 보상기로 흡수.

## 시뮬레이션 결과

### 코악시얼 로터 파라미터 (Table 1)
- `I_x = I_y = 1.383·10⁻³ kg·m²`, `I_z = 2.72·10⁻⁴ kg·m²`
- `k_a = 3.683·10⁻⁵`, `k_β = 3.776·10⁻⁵ N/(rad²·s²)`
- `γ₁ = 1.476·10⁻⁶`, `γ₂ = 1.326·10⁻⁶ N·m/(rad²·s²)`
- `d = 0.0676 m`, `m = 0.41 kg`, `g = 9.81 m/s²`
- 초기조건 `P₀ = [0, 0, 0]^T m`, `η₀ = [0, 0, 0]^T rad`

### 시나리오
1. **Test 1 — Optimization performance**: 사각 경로 (`x_d`, `y_d` 계단형), `z_d = 3 m`, `ψ_d = 1 rad`. OMFFC vs MFFC Set 1, Set 2 (수동 trial-and-error).
2. **Test 2 — Robustness comparison**: 원형 경로 `x_d = 2·sin(0.2t)`, `y_d = 2·cos(0.2t)`, `z_d = 3`, sinusoidal disturbance `h_i = sin(0.1t)·N` at 20s/30s/10s. OMFFC vs PID vs TDEC.
3. **Test 3 — Robustness vs noise & parametric variation**: 8자 경로 `x_d = 2(1-cos(0.2t))`, `y_d = 2·sin(0.4t)`, mass +30% at 10s, `I_x`/`I_y` ±35% at 20s/30s, Gaussian noise (vel σ²=0.002, accel σ²=0.003).

### 정량 결과 (Table 4)

| Criterion | Test 1 OMFFC | Set 1 | Set 2 | Test 2 OMFFC | TDEC | PID |
|-----------|-------------|-------|-------|--------------|------|-----|
| RMSE | 7.51·10⁻⁴ | 2.10·10⁻³ | 2.80·10⁻³ | 5.41·10⁻⁴ | 1.80·10⁻³ | 2.60·10⁻³ |
| MaxAE | 1.50·10⁻¹ | 3.75·10⁻¹ | 2.68·10⁻¹ | 0.81·10⁻¹ | 2.19·10⁻¹ | 3.51·10⁻¹ |

- OMFFC 가 모든 시나리오에서 **RMSE 약 3배 ~ 5배 개선**.
- Test 3: 노이즈·파라미터 변화에도 8자 경로 추종 성공, 자세 응답 Test 1/2 와 유사.

## COAX-SIM 연관

### 본 프로젝트와의 비교
- **COAX-SIM**: model-based 접근 (BSC+DOB, CBF+DOB+BSC, MPC). 정확한 모델 + NDOB/LDOB 로 외란 보상.
- **본 논문**: 완전 model-free, TDE 로 모델 + 외란을 한꺼번에 추정 → 모델 파라미터 의존 0.

### 도입 가능성

#### 1. TDE 를 COAX-SIM 의 DOB 대안 / 보조항으로
- **장점**: 모델 식별 오차에 강건, 구현 단순 (직전 sample 만 필요), 계산량 매우 낮음.
- **단점**:
  - 가속도 측정 `ẋ_{i2}(t-ρ)` 필요 → 수치미분 → 노이즈 증폭. 본 논문은 mathematical differentiation 후 filtering 가정. COAX-SIM 의 IMU/state estimation 품질에 의존.
  - sampling period `ρ` 가 너무 크면 TDE 오차 ↑ (모델이 빠르게 변하면 부정확). COAX-SIM 의 시뮬레이션 step 이 충분히 작아야 함 (예: 1 kHz 이상 권장).
- **의사코드 (COAX-SIM 적용 시나리오)**:
  ```matlab
  % MATLAB pseudo-code (controller loop, one subsystem)
  function u = tde_compensated_control(e, e_dot, y_ddot_d, ...
                                        u_prev, x2_dot_prev, ...
                                        Lambda_P, Lambda_D, Lambda_I, ...
                                        e_int_state)
      % TDE 추정: 직전 입력 - 직전 가속도
      Psi_hat = u_prev - x2_dot_prev;
      % PID-like 항
      u_PID = y_ddot_d + Lambda_P*e + Lambda_D*e_dot + Lambda_I*e_int_state;
      % 합성
      u = u_PID - Psi_hat;
  end
  ```

#### 2. 퍼지 보상기 → P1/P2/P3 적응형 튜닝
- 본 프로젝트의 컨트롤러 게인 (P1, P2, P3) 도 추종 오차/외란 크기에 따라 **적응적으로 조정** 가능.
- 퍼지 규칙: `IF |e| is Large AND |ė| is Large THEN P_gain is High`.
- 단, COAX-SIM 의 CBF 기반 안전제어와 결합 시 퍼지 출력이 CBF constraint 를 위반하지 않도록 saturation 필요.

#### 3. FPA 게인 최적화
- COAX-SIM 의 BSC/CBF 게인 trial-and-error 대체 가능. 다만 6 subsystem × 3 gain = 18 차원 탐색 + sim 100회 반복 → 계산 비용 큼.
- 단순 PSO/GA 와 비교한 우위는 본 논문 외 별도 검증 필요.

### 도입 한계
- COAX-SIM 의 핵심 차별점인 **CBF 안전제어** 와 직접 결합하려면 TDE 의 estimation error bound 가 CBF margin 안에 들어와야 함 → 추가 안정성 해석 필요.
- TDE 는 정상상태 외란 (저주파) 에 강하지만, **고주파 외란 (예: 100 Hz 진동)** 은 직전 sample 과 차이가 크지 않아 추정 실패 → 본 프로젝트의 NDOB 가 더 적합한 경우 존재.

## 핵심 수식 모음 (재현용)

1. **부시스템 모델 (Eq.11)**:
   `ẋ_{i2} = Ψ_i(X, t) + u_i(t)`

2. **TDE 추정 (Eq.18)**:
   `Ψ̂_i(X, t) = u_i(t-ρ) - ẋ_{i2}(t-ρ)`

3. **TDEC 법칙 (Eq.19)**:
   `u_i = ÿ_{id} + Γ_Pi·e_i + Γ_Di·ė_i - u_i(t-ρ) + ẋ_{i2}(t-ρ)`

4. **OMFFC 통합 법칙 (Eq.20)**:
   `u_i = ÿ_{id} + Λ_Pi·e_i + Λ_Di·ė_i + Λ_Ii·∫e_i - Ψ̂_i(X, t) + τ_fi`

5. **퍼지 보상기 (Eq.27)**:
   `τ_fi = -φ̂_i^T·ξ_i(e_i, ė_i) - Δ̂_i·sgn(ε_i^T·P_i·B_i)`

6. **퍼지 파라미터 적응법칙 (Eq.28)**:
   `φ̂̇_i = ϑ_i·ε_i^T·P_i·B_i·ξ_i(e_i, ė_i)`

7. **상한 적응법칙 (Eq.29)**:
   `Δ̂̇_i = η_i·|ε_i^T·P_i·B_i|`

8. **Lyapunov 방정식 (Eq.30)**:
   `P_i·A_i^T + P_i·A_i = -Q_i, Q_i > 0`

9. **안정성 부등식 (Eq.37)**:
   `V̇ ≤ -Σ (1/2)·ϱ_{i,min}(Q_i)·|ε_i|²`

10. **FPA Lévy update (Eq.38)**:
    `S_j^{t+1} = S_j^t + γ·L(λ)·(S* - S_j^t)`

11. **목적함수 (Eq.42)**:
    `J_i = sqrt(Σ_{ℓ=1}^N (y_{id} - y_i)² / N)`

## 한계

### 저자가 언급한 한계
- TDE 가 잘 작동하려면 `Ψ_i(X, t)` 가 sampling period 에 비해 천천히 변해야 함 (Remark 3) → 고주파 변동에 한계.
- 퍼지 보상기의 ideal parameter `φ_i*` 는 universal approximation 가정에 의존 → 실제 시스템에서 보장 어려움.
- FPA 최적화는 오프라인이므로 **실시간 적응** 불가. 미션 중 환경 변화 시 재최적화 필요.
- 향후 실시간 실험 플랫폼 검증 필요 (논문 결과는 numerical simulation 만).
- Fault-tolerant 확장 미구현 (future work 로 언급).

### COAX-SIM 적용 시 주의점
- **가속도 측정 정확도**: TDE 가 `ẋ_{i2}(t-ρ)` 에 매우 민감. IMU 노이즈가 크면 lpf/observer 후처리 필수.
- **Sampling rate**: COAX-SIM 의 control loop 가 1 kHz 이상이 아니라면 TDE 정확도 저하 가능.
- **CBF 결합**: TDE 추정 오차가 CBF safety margin 보다 작아야 함. 형식적 보장 추가 필요.
- **Chattering**: `sgn(·)` 사용 시 입력 진동 → 본 논문처럼 `tanh(·)` 로 치환 권장.
- **퍼지 룰 수 증가** 시 계산량 폭증. 3 rule (N/Z/P) 수준 유지가 실용적.
- 본 논문의 외란은 sinusoidal 저주파 (`sin(0.1t)`) 위주 → COAX-SIM 의 wind gust / payload variation 같은 더 가혹한 시나리오에서 재검증 필요.
