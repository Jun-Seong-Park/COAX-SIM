# CEP 2014 — Gun Launched MAV 계층 Backstepping 제어

## 메타데이터
- **저자:** A. Drouot, E. Richard, M. Boutayeb
- **소속:** Centre de Recherche en Automatique de Nancy, CNRS UMR 7039, Université de Lorraine, France
- **저널:** Control Engineering Practice, Vol. 25 (2014) pp. 16-25
- **DOI:** http://dx.doi.org/10.1016/j.conengprac.2013.11.016
- **수신:** 2013년 6월 29일 / 채택: 2013년 11월 22일 / 온라인 공개: 2014년 1월 5일
- **시스템:** Gun Launched Micro Air Vehicle (GLMAV) — 포(휴대용 무기)에서 발사되는 소형 동축반전 로터 비행체 (질량 ~410 g, 폭 350 mm, 높이 400 mm)
- **키워드:** Micro Aerial Vehicle, Modeling, Hierarchical control, Nonlinear control, Identification

---

## 핵심 기여
1. **GLMAV 개념 검증**: 발사체 형태로 포에서 발사 → 정점에서 로터 전개 → 자율 호버/관측 임무 수행. 발사 에너지를 외부에서 공급함으로써 탑재 에너지를 비행에만 사용.
2. **계층 backstepping 제어기 설계**: Outer loop(translation, 저속) + Inner loop(orientation, 고속)로 시간 척도 분리. 외란이 inner loop에 가해지면 outer loop로 전파되기 전 inner controller가 처리 → 강건성 확보.
3. **적응 식별자 결합**: 미지의 공기역학 효과(외란 F_ext, M_ext)를 상수/완변 가정 하에 Lyapunov 기반 식별자로 추정. Krstic 1995의 backstepping에 Kudva-Narendra/Sheikholeslam 식별기를 결합.
4. **수치/실험 검증**: 비상수(non-constant) 바람 외란 환경에서 수치 시뮬 + OptiTrack 모션캡처 기반 실내 비행 실험으로 추종 성능 입증.

---

## 시스템 모델

### GLMAV 기계 구성 (Section 2)
- 350 mm wing span × 400 mm 높이, ~300 g (프로토타입)
- 상하 2개의 contra-rotating 로터 (gyroscopic 효과 상쇄)
- 두 브러시리스 DC 모터로 구동, **차동 속도**로 yaw 제어, **공통 속도**로 altitude 제어
- 하단 로터에 conventional swashplate 장착 → 2개의 서보모터가 cyclic 입사각 (δ_cx, δ_cy) 조작 → 횡 방향 추력 생성
- 센서: SBG Systems IG-500N IMU (가속도/자이로/지자기 + 기압계 + 4 Hz GPS, EKF 융합) @ 100 Hz
- 임베디드: Gumstix Overo Fire (OMAP3530, Linux) + DSP, ZigBee(원격명령) / WiFi(텔레메트리)

### 좌표계 (Section 3)
- Inertial frame `I`, Body-fixed frame `B`
- Euler 각: η = [φ, θ, ψ]^T (roll, pitch, yaw)
- R_η: body → inertial 회전행렬, Q_η: η̇ ↔ Ω 변환행렬

### 6DOF 강체 동역학 (식 1)
```
ξ̇ = v
m·v̇ = R_η · f
η̇ = Q_η · Ω
J·Ω̇ = -Ω_× · J · Ω + Γ
```
여기서 ξ: 위치, v: 속도(I), Ω: 각속도(B), J: 대각 관성 행렬, Ω_×: 반대칭행렬.

### 공기역학적 추력/모멘트 (식 4-8)
**총 추력 (B frame, 식 4):**
```
T = [Tx; Ty; Tz]
Tx = -β sin(δ_cy) cos(δ_cx) Ω₂²
Ty =  β sin(δ_cx) Ω₂²
Tz = α Ω₁² + β cos(δ_cx) cos(δ_cy) Ω₂²
```
- α < 0, β < 0 : 로터 공기역학 계수
- Ω₁ (상단), Ω₂ (하단) : 로터 회전속도
- δ_cx, δ_cy : 스와시 플레이트 입사각

**모멘트 (식 7):**
```
τ_L =  -d·β·sin(δ_cy)·cos(δ_cx)·Ω₂²
τ_M =   d·β·sin(δ_cx)·Ω₂²
τ_N =   γ₁·Ω₁² + γ₂·Ω₂²        (γ₁>0, γ₂<0)
```
- d : CoG ~ 하단 로터 회전중심 거리

**외란 (논문에서 명시적 분리):**
- F_ext (I frame), M_ext (B frame) — 풍속·드래그 등 측정 불가
- 무게: f_w = m·g·R_η^T·e₃

### 근사 비선형 모델 (식 9-11)
Tx, Ty가 Tz보다 매우 작다는 가정으로 단순화 (식 10):
```
m·v̇ = Tz·R_η·e₃ + m·g·e₃ + F_ext
J·Ω̇ = -Ω_× ·J·Ω + τ + M_ext
```
12개 상태(ξ, v, η, Ω) + 4개 새 입력 (Tz, τ_L, τ_M, τ_N)으로 정리.
역변환 (식 11):
```
Ω₁² = (γ₂·Tz - β·τ_N) / (α·γ₂ - β·γ₁)
Ω₂² = (α·τ_N - γ₁·Tz) / (α·γ₂ - β·γ₁)
δ_cx =  τ_M / (d·β·Ω₂²)        ← paraxial 근사
δ_cy = -τ_L / (d·β·Ω₂²)
```

---

## 계층 제어 구조 (Fig. 4)

```
         ξ_d, ψ_d
            ↓
   ┌──────────────┐   T_z       ┌──────────────┐    ξ, v
   │ Translation  │────────────→│  Translation │──────────→
   │  Controller  │             │   Dynamics   │
   └──────────────┘             └──────┬───────┘
       (Outer loop)                    │
                       φ_d, θ_d        │
            ┌──────────────┐   τ       ↓  η
            │ Orientation  │────────→ ┌──────────────┐
            │  Controller  │          │  Orientation │── η, Ω
            └──────────────┘          │   Dynamics   │
              (Inner loop)            └──────────────┘
```

- **Outer loop**: 위치 ξ_d, yaw ψ_d → 가상 가속도 a_d → 추력 크기 T_z 및 자세 명령 φ_d, θ_d 생성
- **Inner loop**: 자세 η, 각속도 Ω → 토크 τ
- **시간 척도 분리**: orientation dynamics가 translation보다 빠르다는 가정 → high gain backstepping
- **장점**: inner-loop 외란이 outer-loop로 전파되기 전 차단 → 강건성

---

## Backstepping 유도 (Section 4)

논문은 ξ → v → η → Ω 순서로 4단계 backstepping을 수행한다. 4개의 error 변수와 4개의 Control Lyapunov Function (CLF)을 정의한다.

### Step 1 — 위치 error (식 12-17)
```
δ₁ = ξ - ξ_d                                  (12)
V₁ = (1/2) δ₁^T δ₁                            (13)
V̇₁ = δ₁^T (v - ξ̇_d)                          (14)
```
v를 가상 입력으로 보고 desired:
```
v_d = -k_{δ₁} δ₁ + ξ̇_d                       (15)
δ₂ = v - v_d                                  (16)
V̇₁ = -k_{δ₁}||δ₁||² + δ₁^T δ₂                (17)
```

### Step 2 — 속도 error & 가상 가속도 (식 18-30)
```
V₂ = V₁ + (1/2) δ₂^T δ₂                       (18)
V̇₂ = -k_{δ₁}||δ₁||² + δ₂^T δ₂
    + δ₂^T [(1/m)T_zd R_ηd e₃ + g e₃ + (1/m)F̂_ext - v̇_d]
    + (1/m)δ₂^T F̃_ext                         (19)
```
η와 T_z를 두 번째 가상 입력으로 보고 desired 가속도 a_d (식 20):
```
(1/m) T_zd R_ηd e₃ + g e₃ + (1/m) F̂_ext = -δ₁ - k_{δ₂}δ₂ + v̇_d ≜ a_d
```
여기서 η_d = [φ_d, θ_d, ψ_d]^T. ψ_d는 사용자가 정하고 (식 30):
```
a_d = -(k_{δ₁}k_{δ₂}+1)(ξ-ξ_d) - (k_{δ₁}+k_{δ₂})ξ̇_d
      + (k_{δ₁}+k_{δ₂})ξ̇_d + ξ̈_d              (30)
```
**핵심 대입(식 23):** T_z = T_{zd} (직접 설정, no extra dynamics)
```
T_{zd} = -m·sqrt(ã_{dx}² + ã_{dy}² + (ã_{dz}-g)²)   (24)
```
ψ_d ≈ 0, π/2, π, -π/2 의 네 경우마다 φ_d, θ_d 명시 (식 25-28). 예: ψ_d ≈ 0,
```
φ_d = arctan(ã_{dy} / sqrt(ã_{dx}²+(ã_{dz}-g)²))
θ_d = arctan(ã_{dx} / (ã_{dz}-g))
```
ã_d = a_d - (1/m)F̂_ext (식 29).

### Step 3 — 자세 error (식 31-36)
```
ε₁ = η - η_d                                  (31)
V₃ = (1/2) ε₁^T ε₁                            (32)
V̇₃ = ε₁^T (Ω - η̇_d)                          (33)
Q_η Ω_d = -k_{ε₁} ε₁ + η̇_d                   (34)
ε₂ = Q_η Ω - Ω_d                              (35)
V̇₃ = -k_{ε₁}||ε₁||² + ε₁^T ε₂                 (36)
```

### Step 4 — 각속도 error & 실제 입력 토크 (식 37-41)
```
V₄ = V₃ + (1/2) ε₂^T ε₂                       (37)
V̇₄ = -k_{ε₁}||ε₁||² + ε₁^T ε₂
    + ε₂^T (Q̇_η(Ω-Ω_d) + Q_η J⁻¹(-Ω_×JΩ + τ + M̂_ext - Q_η Ω̇_d))
    + ε₂^T Q_η J⁻¹ M̃_ext                       (39)
```
실제 제어 입력 τ (식 40):
```
τ = Ω_× J Ω - M̂_ext + J Q_η⁻¹ (Q_η Ω̇_d - Q̇_η(Ω-Ω_d) - ε₁ - k_{ε₂} ε₂)   (40)
```
```
V̇₄ = -k_{ε₁}||ε₁||² - k_{ε₂}||ε₂||² + ε₂^T Q_η J⁻¹ M̃_ext   (41)
```

**핵심 게인**: k_{δ₁}, k_{δ₂} (translation backstepping), k_{ε₁}, k_{ε₂} (orientation backstepping). 4개 모두 양수.

---

## 식별 구조 (Section 4.2)

F̂_ext, M̂_ext를 위한 Lyapunov 기반 adaptive identifier (Kudva-Narendra 1973, Sheikholeslam 1995). 시스템을 (식 42-44):
```
ẋ = f(x,ζ) + g(x,ζ) u,   f(x,ζ) = f₀(x) + Σ f_i(x)ζ_i,   유사 g.
```
관측기 (식 48):
```
ẋ̂ = A(x̂-x) + w₀(x,u) + w^T Λ⁻¹(x,u) ζ̂
ζ̂̇ = -w(x,u) P (x̂-x) - χ(x)
```
- A = diag(A₁, A₂) ∈ R^{n×n}, Hurwitz (eigenvalues < -σ < 0)
- Λ = diag(Λ₁, Λ₂) > 0 : 적응 게인
- P = diag(P₁, P₂) > 0 : Lyapunov 방정식 A^T P + PA = -Q (Q > 0) 해
- χ(x) : 식별구조 ↔ 계층 제어기 결합 항 (식 57)

**적용 (식 54-57):**
```
x = [x₁; x₂] = [ξ;v; η;Ω],   ζ = [ζ₁; ζ₂] = [F_ext; M_ext]
w₀ : 식 (55)에 명시
w  : 식 (56), block diagonal (1/m)I 와 J⁻¹ 사용
χ  : (1/(2m))Λ₁δ₂, (1/2)Λ₂(ε₂^T Q_η J⁻¹)^T  (식 57) — 제어기 error와 연동
```

---

## 안정성 해석 (Section 4.3)

**확장 Lyapunov 함수:**
```
V₂* = V₂ + (1/2)δ₂^T P₁ δ₂ + x̃₁^T P₁ x̃₁ + ζ̃₁^T Λ₁⁻¹ ζ̃₁     (58)
V₄* = V₄ + (1/2)ε₂^T P₂ ε₂ + x̃₂^T P₂ x̃₂ + ζ̃₂^T Λ₂⁻¹ ζ̃₂     (59)
```
**도함수:**
```
V̇₂* = -k_{δ₁}||δ₁||² - k_{δ₂}||δ₂||² + δ₂^T δ₃ - x̃₁^T Q₁ x̃₁    (60)
V̇₄* = -k_{ε₁}||ε₁||² - k_{ε₂}||ε₂||² - x̃₂^T Q₂ x̃₂              (61)
```
양쪽 모두 negative semi-definite → δ₁, δ₂, ε₁, ε₂ 점근 수렴, 식별 오차는 LaSalle invariance principle로 0 수렴.

**Persistently Exciting 조건 (식 52):**
```
0 < ρ₁I ≤ ∫_t^{t+δ} w(s)w^T(s) ds ≤ ρ₂I   ∀t ≥ 0
```
충족 시 식별기 (식 50)는 globally exponentially stable, 수렴 속도 ≥ min(σ, K), K = ρ₁λ_min(P)/(1 + nρ₂λ_max(P)²) (식 53).

**Cascade 구조:** translation error 동역학(식 62)이 orientation error 동역학(식 63)과 cascade. ε₁ → 0 ⇒ δ₃ → 0 (식 21, 23으로부터) ⇒ M̂_ext → M_ext, F̂_ext → F_ext.

---

## 외란/불확실성 처리

- **방법**: F_ext, M_ext를 **상수 또는 완변 시변(slowly time-varying)** 미지 파라미터로 모델링 → adaptive identifier로 온라인 추정. **DOB 아님**, 매개변수 적응.
- **장점**: parameter convergence를 Lyapunov로 직접 증명.
- **한계**: 빠르게 변하는 외란(고주파 거스트)에는 추적 지연. 시뮬은 비상수 풍속에서도 만족스러운 성능 보였다고만 명시.

---

## 시뮬레이션·실험 결과 (Section 5-6)

### 시뮬레이션 (Section 5, Table 1)
**파라미터:**
| Parameter | Value | Unit |
|-----------|-------|------|
| m | 0.410 | kg |
| g | 9.81 | m/s² |
| J_xx | 1.383e-3 | kg·m² |
| J_yy | 1.383e-3 | kg·m² |
| J_zz | 2.72e-4 | kg·m² |
| α | -3.6835e-5 | N·rad⁻²·s² |
| β | -3.7760e-5 | N·rad⁻²·s² |
| γ₁ |  1.4765e-5 | N·m·rad⁻²·s² |
| γ₂ | -1.3266e-6 | N·m·rad⁻²·s² |
| d | 0.0676 | m |

**게인 (시뮬용, 식별기 + 제어기):**
```
k_{δ₁} = k_{δ₂} = 1.2
k_{ε₁} = 4
k_{ε₂} = 2
```
폐루프 eig는 translation -1.2, orientation -3.
**초기조건**: ξ₀ = [0,0,0.6]^T m, v₀ = 0, η₀ = 0, Ω = 0, ψ_d = 0.
**시나리오**: x, y, z 단계 변화 입력 + 비상수 바람 거스트. Fig. 6: 위치/yaw 추종, Fig. 7: F̂_ext, M̂_ext 추정 vs. 실제 (오차 작음), Fig. 8: Ω₁, Ω₂ 제어 신호.

**Robustness 테스트:**
다음 범위까지 성능 저하 없음:
- 관성 J 불확실성: [-50, 100] %
- 공기역학 (α, β, γ₁, γ₂): [-30, 30] %
- 센싱 노이즈 PSD: [0, 0.005]
- 센서 지연: [0, 50] ms
- 액추에이터 지연: [0, 150] ms
- 샘플링 주기: [0, 300] ms

### 실험 (Section 6)
- **실내 비행**: Université de Technologie de Compiègne, OptiTrack 12-camera mocap → ξ, v 측정(GPS 비활성). η, Ω는 IMU에서 직접.
- **데이터율**: IMU 100 Hz, 제어 100 Hz.
- **보호 아치 추가 → 질량 +100 g** → 수직 방향 추진력 부족 → 트랙 시나리오 단순화 (수평 단계만, 수직 hold).
- **수정된 게인 (식 65):**
  ```
  k_{δ₁} = k_{δ₂} = 1.2 (스칼라)
  k_{ε₁} = diag([45, 45, 10])
  k_{ε₂} = diag([6, 6, 10])
  ```
  → 3×3 대각으로 축별 독립 튜닝, orientation 관련 게인 대폭 상승.
- **결과 (Fig. 10, 11)**: 수평 추종 양호. yaw 추종 일부 저하 (IMU yaw 드리프트 및 mocap 지연 추정). 컨트롤 신호 (Ω₁, Ω₂)는 잡음 많지만 작동.

---

## Singularity 문제 (Section 4.4)

**식 64**:
```
(1/m) T_zd R_{ηd} e₃ + g e₃ + (1/m) F̂_ext = a_d
```
a_d(t) = g·e₃ + (1/m)F̂_ext 일 때 T_zd = 0 → η_d 정의 불능.
- 그 근처에서 |η̇_d|, |η̈_d| → ∞ → τ feedforward 발산 위험.
- 완화 방법: Olfati-Saber 2002 (saturation, 성능 한계), Wood 2007 (dynamic embedding, 본 논문 권장).

---

## COAX-SIM 연관

### 구조적 일치
| 본 논문 (CEP 2014) | COAX-SIM | 일치도 |
|---------------------|----------|--------|
| Translation controller (outer) | `POS2THR.m` → `POS2ATT.m` | 가속도 → 자세 명령 변환 동일 |
| Orientation controller (inner) | `BACKHOCBFQP.m` / `BACKDOB.m` | backstepping 기반 일치 |
| 4-step backstepping (ξ,v,η,Ω) | 위치 PID + 자세 backstepping 2 stage (η,Ω) | COAX-SIM은 위치는 PID 사용 (논문은 backstepping 일관) |
| Lyapunov 게인 k_{δ₁}, k_{δ₂}, k_{ε₁}, k_{ε₂} | K1, K2 (각도/각속도 게인) | 명명만 다름, 역할 동일 |
| Adaptive identifier (F̂_ext, M̂_ext) | `NDOB.m` (nonlinear DOB) | **방법 다름**: 본 논문은 parameter adaptation, COAX-SIM은 NDOB. 둘 다 외란을 ε₂(혹은 ω error) 동역학에서 보상 |
| Singularity at T_z=0 | NED `Tz < -mg` 가정 (호버 부근), 직접 명시 안됨 | COAX-SIM 시나리오는 호버 중심이라 회피됨 |

### 차용 가능 항목
1. **계층 cascade 안정성 증명 구조** (식 60-63, LaSalle invariance) → COAX-SIM 안정성 정당화 자료로 직접 인용 가능
2. **확장 Lyapunov 함수 형태** (V_total = V_control + V_identifier) → NDOB와 backstepping 결합 안정성 증명에 응용 가능
3. **고게인 inner loop + 저게인 outer loop의 시간 척도 분리 정당화**
4. **공기역학 (α, β, γ₁, γ₂)에 ±30% 불확실성 시 성능 유지** 결과 → COAX-SIM의 robustness 벤치마크 기준치로 활용
5. **자세 게인 행렬화 (식 65)**: 축별 독립 게인 (roll, pitch ≠ yaw) → 동축반전 드론의 yaw 모멘트 (γ₁Ω₁² - γ₂Ω₂²)가 roll/pitch와 다른 메커니즘인 점 고려할 때 동일하게 적용 권장

### COAX-SIM과의 시스템 차이점
| 항목 | CEP 2014 GLMAV | COAX-SIM 드론 |
|------|-----------------|-----------------|
| 질량 | 0.41 kg | 11.5 kg |
| 관성 | ~1.4e-3 kg·m² | 0.22 kg·m² |
| 로터 직경 | (~150 mm 추정) | 0.88 m |
| Yaw 방식 | Ω₁/Ω₂ 차동 (동일) | Ω₁/Ω₂ 차동 (동일) |
| Roll/Pitch | Swashplate δ_cx, δ_cy (동일 구조) | Swashplate α, β (동일 구조) |
| 안전 제약 | 없음 (CBF 미사용) | HOCBF 기반 angle_max, omega_max 제약 |
| 발사 조건 | gun-launch 후 정점에서 시작 (큰 초기 각도/spin) | 정지 호버에서 출발 |

**중요**: 본 논문은 호버 후 trim 운용 단계의 제어만 다루며, gun-launch 직후 ballistic phase는 제어 대상이 아니다. 따라서 큰 초기 각도/spin rate 대응 backstepping은 본 논문에서 명시적으로 다루지 않음 (모델 12 상태, 작은 외란 가정).

---

## 핵심 수식 모음 (재현용)

```
(15)  v_d = -k_{δ₁} δ₁ + ξ̇_d
(20)  (1/m) T_zd R_{ηd} e₃ + g e₃ + (1/m) F̂_ext = -δ₁ - k_{δ₂} δ₂ + v̇_d ≜ a_d
(24)  T_{zd} = -m·sqrt(ã_{dx}² + ã_{dy}² + (ã_{dz}-g)²)
(25)  φ_d = arctan(ã_{dy} / sqrt(ã_{dx}² + (ã_{dz}-g)²)),  θ_d = arctan(ã_{dx} / (ã_{dz}-g))   for ψ_d ≈ 0
(29)  ã_d = a_d - (1/m) F̂_ext
(34)  Q_η Ω_d = -k_{ε₁} ε₁ + η̇_d
(40)  τ = Ω_× J Ω - M̂_ext + J Q_η⁻¹ (Q_η Ω̇_d - Q̇_η(Ω - Ω_d) - ε₁ - k_{ε₂} ε₂)
(48)  ẋ̂ = A(x̂-x) + w₀(x,u) + w^T Λ⁻¹(x,u) ζ̂
      ζ̂̇ = -w(x,u) P (x̂-x) - χ(x)
(52)  0 < ρ₁I ≤ ∫_t^{t+δ} w(s) w^T(s) ds ≤ ρ₂I   ∀t ≥ 0   (Persistent Excitation)
(58)  V₂* = V₂ + (1/2) δ₂^T P₁ δ₂ + x̃₁^T P₁ x̃₁ + ζ̃₁^T Λ₁⁻¹ ζ̃₁
(60)  V̇₂* = -k_{δ₁}||δ₁||² - k_{δ₂}||δ₂||² + δ₂^T δ₃ - x̃₁^T Q₁ x̃₁
(61)  V̇₄* = -k_{ε₁}||ε₁||² - k_{ε₂}||ε₂||² - x̃₂^T Q₂ x̃₂
```

---

## 한계 (저자 명시 + COAX-SIM 적용 시 주의)

### 저자가 명시한 한계
1. **외란 가정**: F_ext, M_ext가 상수 또는 완변 시변 → 빠른 거스트에는 추정 지연.
2. **Singularity at a_d ≈ g·e₃ + (1/m)F̂_ext**: T_zd = 0 → η_d 미정의. 해결책 명시 안 함 (Wood 2007 참조 권장).
3. **Paraxial 근사**: δ_cx, δ_cy 작다 가정. 큰 swashplate 입사각에서 모델 오차 누적.
4. **Persistent Excitation 조건 검증 어려움**: 실제 비행에서 w(x,u)가 PE 만족하는지 확인 곤란 → 식별기 수렴 보장 어려움.
5. **실험은 mocap 의존**: GPS 정확도 한계로 옥외 비행 미수행. 향후 작업으로 남김.

### COAX-SIM 적용 시 주의점
1. **시스템 규모 차이**: GLMAV(0.41 kg) vs COAX-SIM(11.5 kg) → 게인 직접 차용 불가. relative scale로 변환 필요.
2. **안전 제약 부재**: 본 논문은 입력 saturation 외 다른 안전 제약 없음. COAX-SIM의 HOCBF angle/ω 제약은 본 논문에 없는 추가 안전층.
3. **DOB vs Adaptive Identifier**: COAX-SIM의 NDOB는 fast disturbance 추정 가능. 본 논문의 식별기는 슬로우-스케일 적응. 빠른 외란 환경에선 NDOB 우세 가능성. 단, parameter convergence 증명은 본 논문 식별기 쪽이 명확.
4. **Outer loop가 PID인 COAX-SIM**: 본 논문처럼 4단계 backstepping 일관 구조로 재설계 시, Lyapunov 게인 (k_{δ₁}, k_{δ₂})까지 통합 튜닝 가능. 안정성 증명도 일관성 확보.
5. **Gun-launch 시나리오 미포함**: 본 논문은 hover/trim 단계만 다룸. COAX-SIM에서 ballistic→hover 천이 시뮬에는 본 논문 그대로 적용 곤란.
6. **Yaw 제어 게인 (식 65)**: 실험에서 yaw 게인을 roll/pitch와 따로 큰 값으로 설정. COAX-SIM도 yaw 게인 별도 튜닝 권장 (γ₁, γ₂가 α, β와 다른 메커니즘이기 때문).
