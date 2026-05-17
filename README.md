# COAX-SIM

**Coaxial Counter-Rotating Rotor Drone — 6DOF MATLAB Simulator**

Backstepping, HOCBF-QP, Disturbance Observer, MPC를 비교 검증하기 위한 동축반전로터 드론 시뮬레이션 프레임워크.

> 박준성 · 김형근, 인천대학교 기계공학과
> 논문: *"안전 제약조건을 만족하는 제어장벽함수 기반 동축반전로터 드론 자세 제어"*

---

## 주요 특징

- **6DOF 비선형 시뮬레이션** — NED 좌표계, body-frame 속도
- **4 가지 제어기** 모드 1-키 전환:
  1. `BACKDOB` — Backstepping + Linear DOB
  2. `BACKHOCBFQP` — Backstepping + HOCBF + QP
  3. `BACKHOCBFQPDOB` — 위 두 가지 결합 (외란 보상 안전 제약)
  4. `MPCctrl` — Model Predictive Control (비교군)
- **3 가지 액추에이터 동역학** — instant / 1차 lag / Schafroth 비선형
- **5 가지 외란 모드** — off / 상수 / 정현파 / 저주파 / 고주파
- **LDOB 옵션** — 자세·위치 외란 추정 + 보상
- **4 가지 시나리오** — hover / box+yaw / circle / circle+heading / RRT CSV
- **3D/2D 애니메이션** 저장, 비교 플롯, 정량 metrics

---

## 디렉토리 구조

```
COAX-SIM/
├── main.m                    ← 메인 실행 (모드 선택 + run_sim 호출)
├── run_sim.m                 ← 시뮬레이션 루프
├── Controller/               ← 제어기 (BSC, HOCBF, MPC, DOB, allocation)
├── Dynamics/                 ← 6DOF, 로터/서보 액추에이터, 외란
├── Trajectory/               ← 시나리오 CSV (waypoints, RRT, obstacles)
├── Scenario/                 ← Scenario0~4.m 시나리오 정의
├── Plot/                     ← plot_results, metrics, obstacles
└── Animation/                ← 3D/2D 애니메이션 (mp4 저장)
```

---

## 빠른 시작

### 요구사항
- MATLAB R2021b 이상
- Toolboxes: Optimization (quadprog), Model Predictive Control (MPCctrl 사용 시), Control System

### 실행
```matlab
>> main
```

[`main.m`](main.m) 상단에서 모드 선택:
```matlab
p.scenario_id    = 3;   % 1: box+yaw  2: circle  3: circle+heading  4: RRT CSV
p.ctrl_mode      = 3;   % 1: BSC+DOB  2: BSC+CBF  3: BSC+CBF+DOB  4: MPC
p.rotor_dyn_mode = 3;   % 1: instant  2: 1차 lag  3: Schafroth + servo lag
p.dis_mode       = 3;   % 0: off  1: const  2: sin(t)  3: sin(0.1t)  4: sin(10t)
p.obs_mode       = 1;   % 1: true state  2: noisy + LDOB
```

실행 후:
- `results.csv` — 시계열 데이터
- `plot_results` — 자세/위치/추력/토크 시계열 figure
- (옵션) `animate_coax(results, p, 'wide', 6, 'sim_result')` — mp4 저장

---

## 좌표·부호 규약

- **NED (North-East-Down)**: x=북, y=동, z=아래 → **고도는 음수** (1 m 고도 = `z = -1`)
- 자세각: ZYX 오일러 (φ roll, θ pitch, ψ yaw), 모든 각도 rad
- 추력 부호: body z 음수 = 위쪽 (양수 = 아래로 미는 것)
- 단위: m, m/s, rad, rad/s, N, N·m, s

상세 규약 및 상태벡터 정의는 [`main.m`](main.m) 와 [Dynamics/sixdof.m](Dynamics/sixdof.m) 참조.

---

## 기본 파라미터 ([main.m](main.m))

| 항목 | 값 |
|------|----|
| 질량 | 11.5 kg |
| 관성 모멘트 | diag([0.2203, 0.2567, 0.1056]) kg·m² |
| 로터 직경 D | 0.88 m |
| CoG ~ 하부 로터 | 0.2 m |
| `cT_hover` / `cQ_hover` | 0.091659 / 0.003523 |
| 추력 간섭 비 (하부/상부) | 0.85 |
| 자세각 한계 (φ, θ) | ±5° |
| 각속도 한계 | ±10°/s (φ, θ), ±30°/s (ψ) |
| 스와시 한계 | ±15° |
| 로터 최대 속도 | 3000 rpm (≈ 314 rad/s) |
| 시뮬 적분 / 제어 주기 | 1 kHz / 500 Hz |

---

## 핵심 알고리즘 요약

### HOCBF 자세 제어 ([Controller/BACKHOCBFQP.m](Controller/BACKHOCBFQP.m))
자세각은 토크에 대해 상대차수 2 → 1차 CBF로는 부족.
HOCBF 시퀀스로 확장 후 QP로 명목 토크를 안전영역으로 투영:

```
b₁ = Θ_max² - Θ²              % 자세각 barrier
b₂ = Θ̇_max² - Θ̇²             % 각속도 barrier
ψ₁ = ḃ₁ + P₁·b₁
ψ₂ = ψ̇₁ + P₂·ψ₁ ≥ 0          % 토크가 등장하는 차수
ψ₃ = ḃ₂ + P₃·b₂ ≥ 0

minimize  ½‖τ - τ_filt‖²
s.t.      A₁·τ ≤ B₁_cond,  A₂·τ ≤ B₂_cond,  -τ_max ≤ τ ≤ τ_max
```

### Backstepping 명목 제어 (모든 모드 공통)
```
Z₁ = Θ - Θ_d
θ_2d = Θ̇_d - K₁·Z₁
Z₂ = Θ̇ - θ_2d
τ_nom = -I·(Z₁ - Θ̈_2d + K₂·Z₂) - d̂      % DOB 보상 포함
```

### LDOB (1차 외란 관측기)
```
d̂/d = L / (s + L)
```
[`Controller/LDOB.m`](Controller/LDOB.m) — 자세, [`Controller/LDOB_pos.m`](Controller/LDOB_pos.m) — 위치.

---

## 결과 정량 지표

`Plot/make_metrics.m` 이 계산:
- 자세각 위반 (논문 Table 3 정의)
- 각속도 위반
- 위치 RMSE
- 추력 / 전력 비 (TPR)
- 제어기 계산시간 (HOCBF vs MPC)

---

## 라이선스

연구·교육용으로 자유 사용. 상업적 재배포 전 저자에게 문의 권장.

---

## 참고 / 인용

본 시뮬레이터의 HOCBF 자세제어는 다음 논문 기반:
- 박준성, 김형근, *"안전 제약조건을 만족하는 제어장벽함수 기반 동축반전로터 드론 자세 제어"*, 인천대학교

동축반전 모델·시스템 식별은 D. Schafroth 등의 muFly 시리즈 (CEP 2010, JIRS 2010) 기반.

---

## 연락처

Jun Seong Park — jsp991204@gmail.com
