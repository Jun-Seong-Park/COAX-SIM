# Park & Kim — HOCBF 기반 동축반전로터 드론 자세 제어

## 메타데이터
- **저자:** 박준성(박사과정), 김형근(부교수, 교신저자 hgkim@inu.ac.kr, ORCID 0000-0001-8492-5876)
- **소속:** 인천대학교 기계공학과 (Department of Mechanical Engineering, Incheon National University)
- **저널:** 한국항공우주학회지 (Journal of The Korean Society for Aeronautical and Space Sciences)
- **발행년도:** 2024년 추정 (Received: January 1, 20XX / Revised: February 1, 20XX / Accepted: March 1, 20XX; 인천대학교 2023년 자체연구비 지원)
- **저작권:** ⓒ 2022 The Korean Society for Aeronautical and Space Sciences
- **키워드:** Urban Air Mobility(도심항공교통), Coaxial Rotor Drone(동축반전로터 드론), Control Barrier Function(제어장벽함수), High Order Control Barrier Function(고차 제어장벽함수)

---

## 핵심 기여
1. **동축반전로터 드론의 비행 성능이 쿼드로터보다 우수함**을 임무 수행시간과 에너지 효율성(TPR) 측면에서 정량적으로 입증.
2. **HOCBF + 외란관측기**를 결합하여 모델 불확실성 환경에서도 자세각·각속도 안전 제약 조건을 만족하는 비선형 제어기 설계.
3. **MPC 대비 평균 계산시간 약 1/2, 최대 계산시간 약 1/10 수준**으로 실시간 안전 제어 가능성 수치 검증 (HOCBF 최대 3.82ms vs MPC 36.96ms).
4. **고차 제어장벽함수의 실현 가능성(feasible safe region)**을 시각화하여 파라미터 p1, p2, p3 선정 근거 제시 (Fig. 3, 4, 5).
5. **상대차수 2 시스템**(자세 각도 → 토크)에서 일반 CBF의 한계를 HOCBF로 극복.

---

## 문제 정의

### 대상 시스템
- 동축반전로터 드론 (2개 로터가 동일 축 상하 배치, 반대 방향 회전)
- 스와시 플레이트 각(α, β)으로 하단 로터 블레이드 피치각 조절 → 추력 방향 제어
- 요 제어: 두 로터 각속도 차이 활용 (테일 로터 불필요)

### 안전 제약
- 자세각 한계: `‖Θ‖ ≤ Θ_max` (식 16)
- 각속도 한계: `‖ω‖ ≤ ω_max` (식 23)
- 토크 제한: `‖T_r‖ ≤ T_r_max`
- 시뮬에서 사용한 값: Θ_max = 2°, ω_max = 5°/s (시나리오 1)

### 비교 대상
- **모델 예측 제어(MPC)**: 자세 및 각속도 제한이라는 동일한 물리적 제약을 제어 설계에 직접 반영 가능 → 공정한 안정성 메커니즘 비교.
- MATLAB `mpcstate`, `mpcmove` 함수 사용.
- 예측 지평선 조합 비교 후 `{N_p=40, N_c=8}` 채택 (응답·계산 성능 최우수).

---

## 시스템 모델

### 강체 동역학 (식 1)
```
m·Ẍ = R(Θ)·T_t + m·g + D_X     (병진)
I·ω̇ = T_r - ω × I·ω + D_φ      (회전)
```
- `m`: 질량, `I = diag([I_x, I_y, I_z])`: 관성 모멘트
- `X = [x, y, z]ᵀ`: NED 좌표계 위치, `Θ = [φ, θ, ψ]ᵀ`: 자세각
- `T_t`, `T_r`: 동체의 추력·제어모멘트
- `D_X`, `D_φ`: 외란항 (외풍 항력, 자이로 모멘트, 모델 불확실성)

**가정:** 작은 자세각·각속도 제한이 있어 롤 피치 코리올리 효과·자이로스코픽 모멘트 무시 가능 → 식 2:
```
I·Θ̈ = T_r
```

### 동축반전 추력·모멘트 (식 3, 4)
```
T_x =  -k₂·Ω₂²·cos(α)·sin(β)
T_y =   k₂·Ω₂²·sin(α)
T_z =  -k₁·Ω₁² - k₂·cos(α)·cos(β)·Ω₂²
τ_φ =  d·T_y
τ_θ = -d·T_x
τ_ψ =  γ₁·Ω₁² - γ₂·Ω₂²
```
- `k_i = C_T·π·ρ·D⁴`: 추력 계수 (i = 1 상단 / 2 하단)
- `γ_i = C_Q·ρ·D⁵`: 모멘트 계수
- `D`: 프로펠러 지름
- `Ω₁, Ω₂`: 상/하 로터 각속도
- `α, β`: 스와시 플레이트 각 (롤/피치 방향)
- `d`: 하단 로터에서 질량 중심까지 거리

**작은 각 가정** (실용적 관점): `cos(α) ≈ cos(β) ≈ 1`, `sin(α) ≈ α`, `sin(β) ≈ β` → T_x, T_y는 T_z 대비 매우 작음.

### NED 좌표계 위치 동역학 (식 5)
```
ẍ = (cφ·sθ·cψ + sφ·sψ)·(T_z + D_x)/m
ÿ = (cφ·sθ·sψ - sφ·cψ)·(T_z + D_y)/m
z̈ = cφ·cθ·(T_z + D_z)/m + g
φ̈ = (d·k₂·Ω₂²·α + D_φ)/I_x
θ̈ = (d·k₂·Ω₂²·β + D_θ)/I_y
ψ̈ = (γ₁·Ω₁² - γ₂·Ω₂² + D_ψ)/I_z
```

### 역변환 (식 6) — 추력/모멘트 → 액추에이터 입력
```
Ω₁² = (γ₂·T_z - k₂·τ_ψ) / (k₁·γ₂ + k₂·γ₁)
Ω₂² = (γ₁·T_z - k₁·τ_ψ) / (k₁·γ₂ + k₂·γ₁)
α   = -τ_φ / (d·k₂·Ω₂²)
β   =  τ_θ / (d·k₂·Ω₂²)
```

### 위치 제어 보조 (식 7, 8)
```
a_d = T_z/m - g

T_z_d = -‖a_d - g‖·m
φ_d   = sin⁻¹[(1/T_z)·(a_x·sψ_d - a_y·cψ_d)]
θ_d   = sin⁻¹[(1/(T_z·cφ_d))·(a_x·cψ_d + a_y·sψ_d)]
```

### 핵심 파라미터 표 (Table 1)
| 파라미터 | Coaxial Rotor | Quad Rotor | 단위 |
|---|---|---|---|
| Mass | 1 | 1 | kg |
| Inertia | [0.026, 0.026, 0.011] | [0.036, 0.036, 0.061] | kg·m² |
| ρ (공기밀도) | 1.225 | 1.225 | kg/m³ |
| D (프로펠러 지름) | 1 | 0.5 | m |
| l (암 길이) | - | 0.35 | m |
| d (로터-CoG 거리) | 0.1 | - | m |
| k_i | 0.0520, 0.0435 | 0.0032 | kg·m |
| γ_i | 0.0073, 0.0069 | 0.0002 | kg·m² |
| Ω_max | 220 | 596 | rad/s |
| a_d_min | [-g·tan(2), -g·tan(2), -2g]ᵀ | (동일) | m/s² |
| a_d_max | [ g·tan(2),  g·tan(2),  g]ᵀ | (동일) | m/s² |

**동일 비교 원칙:** 두 모델은 (1) 1m² 플랫폼 면적 동일, (2) 동일 추력 가정(식 28) → 동일 무차원 추력/토크 계수, 로터 최대 각속도로 조정.
```
T_max = T_coaxial = T_Quad = Σᵢ c_Ti · ρ · D⁴ · Ω_max²   (식 28)
```

---

## 제어기 설계 (논문 핵심)

### 전체 블록 다이어그램 (Fig. 2)
```
Position Controller → Θ_d, T_z_d
   → Attitude Controller (비선형, 명목 토크 T_nom 생성)
   → HOCBF Controller (요구토크 + 현재각도 + 제한값 → 토크 제한 범위)
   → Rotor Controller (역변환: T_r → Ω₁, Ω₂, α, β)
   → Plant
```

### Backstepping 명목 제어기 (T_n)
**Step 1:** Z₁ = Θ - Θ_d (자세 추종 오차)
**가상 제어:** θ_2d = Θ̇_d - K₁·Z₁
**Step 2:** Z₂ = Θ̇ - θ_2d
**Lyapunov 후보:** V = (1/2)·Z₁ᵀZ₁ + (1/2)·Z₂ᵀZ₂
**명목 제어 토크:**
```
T_n = -I·(Z₁ - Θ̈_d + K₁·Ż₁ + K₂·Z₂)
```
- K₁, K₂: 양의 정부호 게인 행렬

> ※ 논문 본문에서는 명시적 Backstepping 수식 번호가 부여되지 않았으나, BACKHOCBFQP.m 코드와 동일한 형태이다.

### HOCBF 안전 제약 유도

#### 일반 HOCBF 이론 (식 9~15)
시스템: `ẋ = f(x) + g₁(x)·u`, `u ∈ U ⊂ ℝ^q`
안전 집합:
```
C = {x ∈ ℝⁿ : b(x) ≥ 0}                  (식 10)
ḃ(x) ≥ -α(b(x)), ∀x ∈ C                  (식 11) → 1차 CBF 조건
L_f b(x) + L_g b(x)·u + α(b(x)) ≥ 0      (식 12)
```
상대차수 m ≥ 2:
```
ψ_i(x) = ψ̇_{i-1}(x) + α_i(ψ_{i-1})       (식 13)
ψ_0(x) = b(x)
C_i = {x : ψ_{i-1}(x) ≥ 0}                (식 14)

L_f^m b(x) + L_g·L_f^{m-1} b(x)·u + S(b(x)) + α_m(ψ_{m-1}) ≥ 0   (식 15)
```

#### 자세각 제한 (3.1절)
**목표:** `‖Θ‖ ≤ Θ_max`
**Barrier 후보 (식 17):**
```
b₁ = Θ_max ∘ Θ_max - Θ ∘ Θ
C₁ = {Θ ∈ ℝ³ : b₁ ≥ 0}
```
여기서 `∘`는 **아다마르 곱** (요소별 곱, MATLAB의 `.*`).

**ψ₁ 정의 (식 18):**
```
ψ₁ = ḃ₁ + α₁(b₁) ≥ 0
   = ḃ₁ + p₁·b₁
```
**Class K 함수 (식 19):** `α_i(·) = p_i(·)`, i ∈ {1, 2, 3}, p_i ∈ ℝ³ˣ³ 양의 정부호 대각.

**ψ₁ 전개 (식 20):**
```
ψ₁ = -2·Θ ∘ Θ̇ + p₁·b₁ ≥ 0
```
→ 식 20에는 제어입력이 직접 나타나지 않음 → ψ₂ 도입.

**ψ₂ 정의 (식 21):**
```
ψ₂ = ψ̇₁ + p₂·ψ₁ ≥ 0
   = -2·(Θ̇ ∘ Θ̇ + Θ ∘ (T_r/I)) + p₂·(-2·Θ ∘ Θ̇ + p₁·b₁) ≥ 0
```
(식 2 `I·Θ̈ = T_r` 사용)

**토크 상한 B₁ (식 22):**
```
B₁ = [(-2·Θ̇ ∘ Θ̇ + p₂·(-2·Θ ∘ Θ̇) + p₁·p₂·b₁) / (2·Θ)] · I
K₁ = {T_r ∈ U : T_r ≤ B₁}
```

#### 각속도 제한 (3.2절)
**목표:** `‖ω‖ ≤ ω_max`
**Barrier 후보 (식 24):**
```
b₂ = ω_max ∘ ω_max - ω ∘ ω
C₃ = {ω ∈ ℝ³ : b₂ ≥ 0}
```

**ψ₃ (식 25):** 상대차수 1 → 1회 미분으로 입력 등장.
```
ψ₃ = ḃ₂ + p₃·b₂ ≥ 0
   = -2·ω ∘ (T_r/I) + p₃·b₂ ≥ 0
```

**토크 상한 B₂ (식 26):**
```
B₂ = (I / (2·ω)) · p₃·b₂
K₂ = {T_r ∈ U : T_r ≤ B₂}
```

#### 파라미터 p1, p2, p3 의미
- `p₁`: 자세각 barrier의 1차 Class K 게인 → 클수록 boundary 접근 시 빠른 감속
- `p₂`: 자세각 barrier의 2차 Class K 게인 → 0 접근 시 토크 제한값 급감
- `p₃`: 각속도 barrier의 Class K 게인 → 각속도 boundary에서 토크 제한 강도
- 권장값: p1 = 100, p2 = 1, p3 = 10 (코드 기준)

#### Class K 함수 선정 근거
**선형 함수** `α_i(·) = p_i(·)` 사용:
- 양의 정부호 → α(0) = 0, 엄격 증가 (Class K 정의 만족)
- 비선형 함수(예: 지수, sigmoid) 대비 QP 형태가 선형 부등식이 되어 quadprog로 효율적 풀이
- 시각화(Fig. 3~5)로 실현 가능 영역 확인 가능

#### 실현 가능성 분석 (3.4절, Fig. 3~5)
- **Fig. 3:** ψ₁ ≥ 0 만족 파라미터 영역 (-2° ≤ φ ≤ 2°, -5°/s ≤ ω_φ ≤ 5°/s). 예: p₁ = 100, φ = 2°일 때 제한 각속도 ω_φ = 0°/s (안전 영역 내).
- **Fig. 4:** ψ₂ ≥ 0 시각화, p₁ = 100 일 때 ω_φ = 5°/s 인 경우 제어 모멘트 τ_φ = -0.21 N·m 보다 작은 입력 사용 시 실현 가능.
- **Fig. 5:** ψ₃ ≥ 0 시각화 (식 24 기반). ω_φ = 5°/s 일 경우에도 경계 내 → 실현 가능.
- **결론:** "동적 장애물이 다가오는 것이 아닌 고정된 상태를 벗어나지 않는 문제" → 단순히 큰 제어입력 사용을 회피하면 실현 가능.

### QP 정식화 (3.3절, 식 27)
```
T_r = argmin (1/2)·‖T_r - T_n‖²
       T_r ∈ ℝ³ˣ¹
  s.t. T_r ≤ B₁    (자세각 안전)
       T_r ≤ B₂    (각속도 안전)
```
- **결정변수:** T_r ∈ ℝ³
- **목적함수:** 명목 백스테핑 토크 T_n과의 거리 최소화 (성능 유지)
- **제약:** B₁, B₂ 동시 만족 (안전 확보)
- **풀이:** MATLAB `quadprog`의 interior-point 방법

#### Hessian/Gradient 구조 (코드 매핑용)
표준형 `min (1/2)·xᵀHx + fᵀx`:
```
H = 2·I (3x3 단위행렬의 2배) = diag([2, 2, 2])
f = -2·T_n  (또는 코드에서 -2·T_filt_prev)
A·T_r ≤ b 의 A = [diag(A₁); diag(A₂)],  b = [B₁; B₂]
  A₁ = 2·Θ,  A₂ = 2·Θ̇
lb = -T_r_max,  ub = +T_r_max
```

---

## 시뮬레이션 설정

### 환경
- CPU: Intel Core i7-12700
- RAM: 16 GB
- Solver: MATLAB `quadprog` (interior-point), MPC: `mpcstate` + `mpcmove`
- 제어 루프: PX4 표준 **400 Hz** (dt_ctrl ≈ 2.5 ms)

### 시나리오 1 — 각도 제한 제어기 성능평가
- **목표:** 자세 명령 `5·sin(t)` [deg] 추종, 자세 ≤ 2°, 각속도 ≤ 5°/s 만족 확인
- **초기 조건:** 자세각·각속도 모두 0
- **결과:** Fig. 6 (각도), Fig. 7 (각속도), Fig. 8 (제어 토크), Fig. 9 (barrier 크기), Fig. 10 (Barrier 1 vs 토크), Fig. 11 (Barrier 2 vs 토크)
- **관찰:** 0.4초 부근에서 b₁ → 0 접근 시 토크 제한값 급감 → 명목 입력보다 낮게 결정. 3.4초 부근에서 b₁이 다시 0에 접근 시 제한값을 상회하는 제어입력 사용 (직관: 상한선 +2° 접근 시 입력 낮춤, 하한선 -2° 접근 시 더 큰 입력 사용).

### 시나리오 2 — 회전익 무인기 모델 성능 비교 (CoaX vs Quad)
- **임무:** Table 2의 waypoint 순차 추종 (수직 가속도 응답 차이 부각).
- **waypoint 도달 기준:** 유클리드 거리 0.1 m 이내.
- **결과 (Table 2):**

| waypoint | 1 | 2 | 3 | 4 | 5 | 6 | mean TPR |
|---|---|---|---|---|---|---|---|
| position | [0,0,-1] | [5,0,-3] | [-5,5,-1] | [0.5,5,-2] | [0,0,-1] | [-0.6,0,0] | - |
| Quad [s] | 6.6 | 18.6 | 30.4 | 42.3 | 54.1 | 60.8 | 0.65 |
| CoaX [s] | 5.4 | 12.7 | 19.8 | 26.9 | 33.8 | **39.7** | **0.51** |

> 주의: 본문은 "전체 구간 효율은 연구 모델 0.65, 비교 모델 0.51"이라고 명시 → Table 2의 mean TPR 열 값과 행 라벨이 반대 매핑일 가능성 (논문 원문 검수 필요). 본문 텍스트가 우선이라면 **CoaX TPR = 0.65, Quad TPR = 0.51 (효율 차 27.52%)**.

- **결론:** 동축반전 모델이 임무 수행시간 약 20초 단축, TPR 27.52% 우수 → 도심항공교통(UAM)에 더 적합.
- **그림:** Fig. 12 (3D 궤적), Fig. 13 (위치 응답), Fig. 14 (가속도 응답), Fig. 15 (자세 응답), Fig. 16 (TPR).

### 시나리오 3 — CBF vs MPC w/o Disturbance
- **임무:** 시나리오 2와 동일 경로점.
- **MPC 설정:** {N_p=40, N_c=8} 채택 (테스트한 조합: {20,3}, {20,8}, {40,3}, {40,8}).
- **결과:** 두 제어기 모두 안정적 자세 추종, 자세·각속도 제약 만족.
- **그림:** Fig. 17 (위치 응답), Fig. 18 (자세 응답), Fig. 19 (각속도), Fig. 20 (제어 토크), Fig. 21 (제어 입력).

### 시나리오 4 — CBF vs MPC w/ Disturbance + 외란관측기 (DOB)
- **외란 모델:** 정현파 토크 외란
  - 크기: **0.025 N·m** (최대 제어 모멘트 0.5 N·m 대비 약 5% → 일반 항공 제어 5~10% 외란 가정의 보편적 수준)
  - 주기: **2π 초**
- **외란관측기 (식 30~33):**
  ```
  ẋ = f(x) + g₁(x)·u + g₂(x)·d                       (식 30)
  ż = -l(x)·[g₂(x)·(z - p(x)) + f(x) + g₁(x)·u]
  d̂ = z + p(x)                                       (식 31)
  l(x) = ∂p(x)/∂x                                    (식 32)
  ```
  자세 제어 적용 (식 33), 사용 이득 L = 1:
  ```
  ż = -l·[z + p(x) + τ_d]/I
  p(x) = l(x)·ω
  ```
- **DOB 결합 ψ₃ (식 34):**
  ```
  ψ₃ = ḃ₂ + p₃·b₂ ≥ 0
     = -2·ω ∘ (T_r + d - d̂)/I + p₃·b₂ ≥ 0
  ```
  → 외란 존재 상황에서도 ψ₃ ≥ 0 보장.
- **결과 그림:** Fig. 22 (위치), Fig. 23 (자세), Fig. 24 (각속도, MPC만 채터링 발생, HOCBF는 없음), Fig. 25 (제어 토크), Fig. 26 (제어 입력), Fig. 27 (외란관측기 추정 결과).
- **MPC 채터링 원인:** 동축반전 모델의 설계 특성상 최대 제어 모멘트가 추력 변화에 따라 실시간 변동 → MPC는 수평 비행 시 한정된 최대 모멘트를 가정 → 실제 환경에 도달 불가능한 명령 산출.

---

## 결과 분석 — Table 3 (시나리오 3 & 4 정량 결과)

| Metric | Unit | Scn.3 HOCBF | Scn.3 MPC | Scn.4 HOCBF | Scn.4 MPC |
|---|---|---|---|---|---|
| Average Computation Time | ms | 0.028 | 0.014 | 0.029 | 0.0154 |
| **Maximum Computation Time** | **ms** | **2.99** | **35.74** | **3.82** | **36.96** |
| Angle Barrier Violation Sum (Roll, Pitch, Yaw) | deg² | 0.00 / 0.00 / 0.00 | 0.29 / 0.16 / 0.00 | 0.20 / 0.00 / 0.00 | 23.52 / 9.64 / 0.00 |
| Angular Velocity Barrier Violation Sum (Roll, Pitch, Yaw) | deg²/s² | 0.00 / 0.00 / 0.00 | 0.00 / 0.00 / 0.00 | 0.00 / 0.01 / 0.00 | 0.80 / 0.29 / 0.00 |

### 핵심 수치 해석
- **계산 시간:** 평균은 MPC가 빠르지만 (단일 step의 단순 형태일 때), HOCBF는 평균·최대 모두 **제어 주기 2.5 ms 내** 안정적. **MPC 최대는 HOCBF 대비 1095.32% (Scn.3), 867.54% (Scn.4) 큼** → 실시간 제어 보장 불가.
- **이유:** MPC는 다중 step 최적화, HOCBF는 단일 step 제어장벽함수 최적화.
- **제약 위반 (시나리오 3 무외란):**
  - HOCBF: 모든 항목 0
  - MPC: 롤 각도 0.29 deg², 피치 각도 0.16 deg² 위반
- **제약 위반 (시나리오 4 외란):**
  - HOCBF: 롤 각도 0.20 deg², 피치 각속도 0.01 deg²/s² (미미)
  - MPC: 롤 각도 **23.52 deg²**, 피치 각도 9.64 deg², 롤 각속도 0.80 deg²/s², 피치 각속도 0.29 deg²/s² (심각)
- **종합:** HOCBF가 외란 환경에서 제약 위반 합계가 현저히 적고 실시간 안전 제어기로 우수성 입증.

---

## COAX-SIM 코드 매핑

### 논문 ↔ BACKHOCBFQP.m 줄별 대응

| 논문 수식 | 코드 위치 (BACKHOCBFQP.m) | 비고 |
|---|---|---|
| Backstepping Z₁ = Θ - Θ_d | L41: `Z1 = Theta - Theta_d` | |
| Z₁̇ = Θ̇ - Θ̇_d | L42: `Z1_dot = Theta_dot - Theta_d_dot` | |
| 가상 제어 θ_2d | L44: `theta_2d = Theta_d_dot - K1 * Z1` | |
| Z₂ = Θ̇ - θ_2d | L45: `Z2 = Theta_dot - theta_2d` | |
| Θ̈_2d = Θ̈_d - K₁·Ż₁ | L46: `Theta_2d_dd = Theta_d_ddot - K1 * Z1_dot` | |
| 명목 토크 T_n | L49: `Torque_nom = -p.Inertia * (Z1 - Theta_2d_dd + K2 * Z2)` | 부호 정의 일치 |
| 1차 LPF (필터) | L52: `Torque_filt_prev = ... (Torque_nom - Torque_filt_prev)/tau_filt` | 논문엔 없음, 채터링 방지용 |
| **b₁ (식 17)** | L60: `B1 = Theta_max.^2 - Theta.^2` | `.^2`로 아다마르 자승 |
| **b₂ (식 24)** | L61: `B2 = Theta_dot_max.^2 - Theta_dot.^2` | |
| ψ₂ A₁ 계수 = 2Θ | L63: `A1_cond = 2 * Theta` | |
| ψ₃ A₂ 계수 = 2Θ̇ | L64: `A2_cond = 2 * Theta_dot` | |
| **B₁ (식 22)** | L65: `B1_cond = p.Inertia * (-2*Theta_dot.^2 + (P1+P2)*(-2*Theta.*Theta_dot) + P1*P2*B1)` | I*(...) 형태로 토크 상한 |
| **B₂ (식 26)** | L66: `B2_cond = p.Inertia * P3 * B2` | I*p₃*b₂ |
| 요축 b₁ 제거 | L69-70: `A1_cond(3)=0; B1_cond(3)=1` | yaw에는 자세각 barrier 미적용 (실용적 처리) |
| **QP 식 27** | L73-83: `quadprog(H, f, A, b, [], [], lb, ub, ...)` | H = diag(2,2,2), f = -2·T_filt |
| 토크 한계 (lb, ub) | L79-80: `lb = -Torque_max, ub = Torque_max` | |
| Warm start | L83: 초기값 `Torque_prev` | infeasible 회피 도움 |

### 논문 파라미터 ↔ MATLAB 파라미터

| 논문 기호 | MATLAB 변수 | 형태 | 권장값 |
|---|---|---|---|
| p₁ | `p.P1_cbf` | 스칼라 → P1 = p.P1_cbf * eye(3) | 100 |
| p₂ | `p.P2_cbf` | 스칼라 → P2 = p.P2_cbf * eye(3) | 1 |
| p₃ | `p.P3_cbf` | 스칼라 → P3 = p.P3_cbf * eye(3) | 10 |
| K₁ | `p.K1_bsc` (diag) | 벡터 → diag | 백스테핑 1단 |
| K₂ | `p.K2_bsc` (diag) | 벡터 → diag | 백스테핑 2단 |
| I | `p.Inertia` | 3x3 행렬 | diag([0.026, 0.026, 0.011]) |
| Θ_max | `state_max(:,1)` | rad | [2; 2; 2]·π/180 |
| ω_max | `state_max(:,2)` | rad/s | [5; 5; 5]·π/180 |
| T_r_max | `state_max(:,3)` | N·m | 시뮬에서 동적 계산 |
| τ_filt | `p.tau_filt` | s | 0.02 (논문엔 명시 없음) |

### 재현 시 주의점
1. **요(yaw) 처리:** 논문은 모든 축에 자세각 barrier 적용. 코드는 yaw 제외 (L69-70). 도심항공교통 임무에서 yaw는 크게 변동 → 실용적 절충. **논문 재현 시 yaw도 포함하려면 두 줄 주석 처리**.
2. **명목 토크 부호:** 논문 `T_n = -I·(...)` 형태 → 코드도 동일.
3. **단위:** 시뮬은 rad/s 사용. 결과 표기 시 deg/s로 변환.
4. **DOB 결합 (식 34):** BACKHOCBFQPDOB.m 별도 구현. 외란 추정 d̂ 를 B₂_cond에서 차감해야 함.
5. **MPC 비교군:** MATLAB Model Predictive Control Toolbox 필요. Np=40, Nc=8 설정.
6. **외란 모델 (시나리오 4):** 정현파 토크 0.025 N·m, 주기 2π s. `Environment/DisGen.m`의 정현파 모드 사용.
7. **TPR 계산 (식 29):**
   ```
   η = T/P = Σ C_T·ρ·π·D⁴·Ω² / Σ 2π·C_Q·ρ·D⁵·Ω³
   ```

---

## 한계 및 향후 과제

### 저자 명시 한계
- HOCBF 적용 시 파라미터 값에 따라 **안전 집합에 대한 해(feasible control input)가 존재하지 않는 경우 발생** → 본 연구는 시각화(Fig. 3~5)로 실현 가능 영역을 보장하여 해결.
- 본 논문은 **자세 제어**에 한정. 위치 안전 제약은 다루지 않음.

### 잠재적 확장
1. **위치 안전 제약 HOCBF:** 장애물 회피 (DOCBF, C3BF 등과 결합).
2. **DOB + HOCBF 통합 정식화:** 본 연구는 식 34에서 d̂를 ψ₃에 단순 차감. 본격적인 추정 오차 bound를 포함한 robust HOCBF로 확장 가능.
3. **시변 Class K 함수 α(·, t):** 논문은 시간 불변 가정. 임무 단계별 boundary 강도 조절 가능.
4. **비선형 외란 모델:** 정현파 외란 외에 임의 형태 외란에 대한 추정 성능 평가.
5. **하드웨어 실증:** MATLAB 시뮬 → PX4 실제 비행 검증.

---

## 핵심 수식 모음 (재현용)

### (1) 강체 동역학
```
m·Ẍ = R(Θ)·T_t + m·g + D_X
I·ω̇ = T_r - ω × I·ω + D_φ
```

### (2) 자세 단순화 동역학
```
I·Θ̈ = T_r
```

### (3) 추력 벡터
```
T_t = [T_x; T_y; T_z]ᵀ
    = [-k₂·Ω₂²·cα·sβ;  k₂·Ω₂²·sα;  -k₁·Ω₁² - k₂·cα·cβ·Ω₂²]ᵀ
```

### (15) HOCBF 일반형
```
L_f^m b(x) + L_g·L_f^{m-1} b(x)·u + S(b(x)) + α_m(ψ_{m-1}(x)) ≥ 0
```

### (17) 자세각 안전 집합
```
b₁ = Θ_max ∘ Θ_max - Θ ∘ Θ
C₁ = {Θ : b₁ ≥ 0}
```

### (18) ψ₁
```
ψ₁ = ḃ₁ + p₁·b₁ ≥ 0
```

### (21) ψ₂ (제어입력 등장)
```
ψ₂ = -2·(Θ̇ ∘ Θ̇ + Θ ∘ (T_r/I)) + p₂·(-2·Θ ∘ Θ̇ + p₁·b₁) ≥ 0
```

### (22) 자세각 토크 상한
```
B₁ = [(-2·Θ̇ ∘ Θ̇ + p₂·(-2·Θ ∘ Θ̇) + p₁·p₂·b₁) / (2·Θ)] · I
```

### (24)(25)(26) 각속도 barrier → 토크 상한
```
b₂ = ω_max ∘ ω_max - ω ∘ ω
ψ₃ = -2·ω ∘ (T_r/I) + p₃·b₂ ≥ 0
B₂ = (I / (2·ω)) · p₃·b₂
```

### (27) QP 최종 정식화
```
T_r = argmin (1/2)·‖T_r - T_n‖²
        T_r
  s.t. T_r ≤ B₁
       T_r ≤ B₂
```

### (28) 동일 추력 조건
```
T_max = Σᵢ C_Ti · ρ · D⁴ · Ω_max²
```

### (29) 추력 대비 전력비 (TPR)
```
η = T/P = Σ C_T·ρ·π·D⁴·Ω² / Σ 2π·C_Q·ρ·D⁵·Ω³
```

### (31)(33) 비선형 외란관측기 (자세 적용)
```
ż = -l·[z + p(x) + τ_d] / I
p(x) = l(x)·ω
d̂ = z + p(x)
```

### (34) DOB 결합 ψ₃
```
ψ₃ = -2·ω ∘ ((T_r + d - d̂)/I) + p₃·b₂ ≥ 0
```

---

## 정의·정리 (부록)

- **[정의 1] 전방 불변성 (Forward Invariance):** 집합 C ⊂ ℝⁿ 이 시스템에 대해 전방 불변이면 `x(t₀) ∈ C ⇒ x(t) ∈ C, ∀t ∈ [t₀, t_f]`.
- **[정의 2] 클래스 K 함수 (Class K Function):** Lipschitz 연속 함수 `α: [0, a) → [0, ∞)`, a > 0가 엄격히 증가하고 `α(0) = 0`이면 클래스 K 함수.

---

## 참고문헌 (논문 본문에서 인용된 핵심 항목)
- [3] Cornelius et al., "Rotor performance predictions for urban air mobility: Single vs. coaxial rigid rotors," Aerospace, 2024.
- [19] Ames et al., "Control barrier functions: Theory and applications," ECC 2019.
- [20] Xiao & Belta, "High-order control barrier functions," IEEE TAC 2021.
- [21] Taylor et al., "Safe backstepping with control barrier functions," CDC 2022.
- [22] Xiao et al., "Sufficient conditions for feasibility of optimal control problems using control barrier functions," Automatica 2022.
- [23] Goli et al., "Experimental study on efficient propulsion system for multicopter UAV design applications," Results in Engineering 2023.
- [24] Chen, "Disturbance observer based control for nonlinear systems," IEEE/ASME Trans. Mechatronics 2004.
