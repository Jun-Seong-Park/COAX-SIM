# ISAT 2016 — ESO 기반 동축반전 UAV 제어

## 메타데이터
- **저자:** M. Rida Mokhtari, Amal Choukchou Braham, Brahim Cherki
- **소속:** Laboratoire d'Automatique de Tlemcen (LAT), Electrical Engineering Department, Tlemcen University, Algeria
- **저널:** ISA Transactions, Vol. 61 (2016), pp. 1–14
- **DOI:** http://dx.doi.org/10.1016/j.isatra.2015.11.024
- **수신/수정/게재:** 2015-06-28 수신 / 2015-11-09 수정 / 2015-11-25 게재 승인 / 2015-12-18 온라인 공개
- **키워드:** Extended State Observer (ESO), Active Disturbance Rejection Control (ADRC), Hierarchical control, Coaxial-rotor
- **타입:** Research Article, Elsevier ISA Transactions

## 핵심 기여
1. **미지의 가변 공력 외란(unknown variable aerodynamic efforts)을 다루는 제어 구조** 도입. 풍속·시간변동 외란 모두 포함.
2. **계층(Hierarchical) 비행 제어 구조**: cascade property 활용해 translational 부시스템(outer loop)과 rotational 부시스템(inner loop)으로 분리.
3. **LESO 기반 선형 제어기(linear control + LESO)**로 강건성 확보. NESO 대신 LESO를 채택해 pole placement로 이득 체계화.
4. **closed-loop 시스템(control + observer)의 수렴성을 Lyapunov로 정식 증명** — observer error는 bounded ball $B_r$로 지수수렴, closed-loop 오차도 마찬가지.
5. 일반적 ADRC는 underactuated 시스템에 직접 적용 불가하지만, 본 논문은 cascade 분해로 ESO를 underactuated coaxial-rotor에 적용.

## 시스템 모델

### 대상 기체
- **CRUAV (Coaxial-Rotor UAV)** — 두 개의 brushless DC 모터로 회전하는 두 main rotor(상·하). 반대 방향 회전 → gyroscopic / drag torque 상쇄.
- 하부 rotor에 **cyclic swashplate** + 2 servo motor → roll/pitch torque 생성(thrust vectoring).
- 4개 제어 입력으로 6 DoF 중 4 DoF 제어: thrust $T$ + 3축 control torque $\Gamma = (\tau_\phi, \tau_\theta, \tau_\psi)^T$.

### 동역학 모델 (Newton–Euler)
$$
\begin{cases}
m\ddot{\xi} = R_\eta T - m g z_e + F_{ext} \\
M(\eta) \dot{w} = \Psi_\eta^T \Gamma - C(\eta, \dot\eta) w + \Gamma_{ext}
\end{cases}
$$
- $\xi = (x,y,z) \in \mathbb{R}^3$: inertial 위치, $\eta = (\phi, \theta, \psi)^T$: Euler 각.
- $M(\eta) = \Psi_\eta^T J \Psi_\eta$: auxiliary positive inertia matrix.
- $F_{ext}, \Gamma_{ext}$: 모든 미지의 공력 외란.

### 추력/토크 생성식 (eq. 3,4)
- Thrust vector: $T = (-\kappa_\beta \sin\delta_{cy} \cos\delta_{cx}\,\Omega_2^2,\ -\kappa_\beta \sin\delta_{cx}\,\Omega_2^2,\ \kappa_\alpha \Omega_1^2 + \kappa_\beta \cos\delta_{cx} \cos\delta_{cy}\,\Omega_2^2)^T$
- Torque: $\tau_\phi = -d\kappa_\beta \sin\delta_{cx} \cos\delta_{cy}\,\Omega_2^2$, $\tau_\theta = d\kappa_\beta \sin\delta_{cy} \cos\delta_{cx}\,\Omega_2^2$, $\tau_\psi = \gamma_1 \Omega_1^2 - \gamma_2 \Omega_2^2$.

### 가정
- **Assumption 1**: $-\pi/2 \leq \phi(t), \theta(t) \leq \pi/2$.
- **Assumption 2**: 기준 궤적과 1차·2차 도함수 bounded.
- **Assumption 3,4**: 외란 변화율 $p(\zeta), \Delta(\bar\zeta)$ bounded(단, 절대값 자체는 unknown).
- **Remark 1**: 작은 swashplate 각도 가정으로 $\Sigma \approx 0$, 측면 small body force 무시(diffeomorphism).

## ESO 설계

### LESO vs NESO — 본 논문은 LESO 사용
**Remark 3**의 LESO 선정 사유:
- 이득을 **pole placement로 체계적으로 선택 가능** (NESO는 trial-and-error)
- **closed-loop 안정성을 수렴적으로 증명 가능**
- **하드웨어 구현이 쉬움**

### ESO 차수 (relative degree)
- Translational error subsystem $\delta_1 \in \mathbb{R}^6$ → 2차 더블 인테그레이터 + 외란 → **3차 LESO** ($\zeta_1, \zeta_2, \zeta_3$).
- Rotational error subsystem $\delta_2 \in \mathbb{R}^6$ → 동일 구조 → **3차 LESO** ($\bar\zeta_1, \bar\zeta_2, \bar\zeta_3$).
- 두 LESO는 각각 9차 augmented 상태 ($\mathbb{R}^9$).

### Translational LESO (eq. 21)
$$
\begin{cases}
\dot{\hat\zeta}_1 = \hat\zeta_2 + \frac{\alpha_1}{\epsilon} I (\zeta_1 - \hat\zeta_1) \\
\dot{\hat\zeta}_2 = \mu_\xi + \frac{\alpha_2}{\epsilon^2} I (\zeta_1 - \hat\zeta_1) \\
\dot{\hat\zeta}_3 = \frac{\alpha_3}{\epsilon^3} I (\zeta_1 - \hat\zeta_1)
\end{cases}
$$
- $\alpha_i$: observer gains, $\epsilon$: small positive scaling (관측기 대역폭 ~ $1/\epsilon$).
- 확장 상태 $\zeta_3 = \dot\Delta(h, F_{ext})/(...)$ — 총 외란(total disturbance) 추정.

### Rotational LESO (eq. 41)
- 동일한 구조, gain matrix $L \in \mathbb{R}^{9\times 3}$.

### 이득 선정
- $L_x = [\alpha_1 I, \alpha_2 I, \alpha_3 I]^T$, $A_0 - L_x C$ Hurwitz 조건.
- **Pole placement**: $A_0 - L_x C$의 고유치를 원하는 위치(예: $-\omega_o$ 다중극)에 배치.
- $\epsilon$ 작을수록 외란 추정 정확도 증가, 그러나 잡음/주파수 trade-off.

## 제어기 구조

### 계층 구조 (Fig. 3, Fig. 4)
**Outer loop (Translation Controller, eq. 23)**: 위치 추적
- Virtual control: $\mu_\xi = q(T_z, \phi_d, \theta_d) = \frac{1}{m} T_z R_\eta^d z_e$ (eq. 12)
- desired thrust + desired Euler 각도 추출 (eq. 13):
  $$T_z = m\sqrt{\mu_x^2 + \mu_y^2 + (\mu_z+g)^2}$$
  $$\phi_d = \sin^{-1}\left[\frac{m}{T_z}(\mu_x \sin\psi_d - \mu_y \cos\psi_d)\right]$$
  $$\theta_d = \sin^{-1}\left[\frac{m}{T_z \cos\phi_d}(\mu_x \cos\psi_d + \mu_y \sin\psi_d)\right]$$
- LESO 보상 포함 제어 법칙: $\mu_\xi = \ddot{\xi}_d - K_\xi(\hat\zeta_1 - \xi_d) - K_{v_\xi}(\hat\zeta_2 - \dot\xi_d) + u_{oc}$ with $u_{oc} = -\hat\zeta_3$ (eq. 23).

**Inner loop (Attitude Controller, eq. 42)**:
- Feedback linearization 변환: $\Gamma = J \Psi_\eta \tilde\Gamma + \Phi_\eta^T C(\eta, w) w$ (eq. 10), 여기서 $\Phi_\eta = \Psi_\eta^{-1}$.
- 제어 법칙: $\tilde\Gamma = \ddot{\eta}_d - K_\eta(\bar\zeta_1 - \eta_d) - K_w(\bar\zeta_2 - \dot\eta_d) + \Gamma_{oc}$, $\Gamma_{oc} = -\hat{\bar\zeta}_3$ (eq. 42).

### Non-linear State Error Feedback?
- 본 논문은 **LESO** 사용으로 nonlinear state error feedback은 도입하지 않음. 단순 선형 PD형 + 외란 보상 항.

### 외란 보상항
- Outer loop: $u_{oc} = -\hat\zeta_3$ (translational total disturbance 추정치를 직접 뺌)
- Inner loop: $\Gamma_{oc} = -\hat{\bar\zeta}_3$ (rotational total disturbance 추정치를 직접 뺌)

## 안정성 해석

### Theorem 1 — Translational observer error
- Lyapunov: $V_0 = \rho^T P_0 \rho$, $\rho = [\bar\rho_1, \bar\rho_2, \bar\rho_3]^T$, $\bar\rho_i(t) = \tilde\zeta_i(\epsilon t)/\epsilon^{n+1-i}$.
- $(A_0 - L_x C)^T P_0 + P_0(A_0 - L_x C) = -I_{9\times 9}$
- $\dot V_0 \leq -\|\rho\|^2 [1 - 2\epsilon \lambda_{\max}(P_0) h_{\max}/\|\rho\|]$
- $\|\rho\| > 2\epsilon \lambda_{\max}(P_0) h_{\max}$이면 $\dot V_0 < 0$ → bounded ball $B_{r_1}$로 지수수렴.
- $\epsilon \to 0$이면 ball 반경 → 0.

### Theorem 2 — Translational closed-loop
- $V_{cl} = \delta_{cl}^T P_{cl} \delta_{cl}$, $A_{cl}^T P_{cl} + P_{cl} A_{cl} = -I$
- $K_1, L_x$가 $A_{cl}$ Hurwitz 만들면 $\delta_{cl}$도 bounded ball $B_{r_2}$로 지수수렴.

### Theorem 3, 4 — Rotational observer/closed-loop
- 동일 구조 결과.

### Remark 6 — coupling
- 두 부시스템은 interconnection term $h(\eta_d, \delta_\eta)$로 연결: $\|h\| \leq k_2 \|\delta_2\|$ (eq. 직전).
- rotational $\delta_2 \to 0$이 translational $\delta_1 \to 0$을 함의 → cascade 안정성 보장.

### 결론 closed-loop 시스템 (eq. 55)
$$
\begin{cases}
\dot\delta_1 = A_{1u} \delta_1 + [B_1 K_1 \ B_1] \epsilon^{3-i} \rho \\
\dot\delta_2 = A_{2u} \delta_2 + [B_2 K_2 \ B_2] \epsilon^{3-i} \bar\rho
\end{cases}
$$
→ 추적 오차들이 작은 bounded ball로 지수수렴.

## 시뮬레이션 결과

### 기체 파라미터
- $m = 0.41$ kg, $g = 9.81$ m/s², $d = 0.0676$
- $J = \mathrm{diag}(1.383, 1.383, 0.272) \times 10^{-3}$ kg·m²
- $\kappa_\alpha = 3.6835 \times 10^{-5}$ N·rad⁻²·s², $\kappa_\beta = 3.7760 \times 10^{-5}$
- $\gamma_1 = 1.4765 \times 10^{-6}$, $\gamma_2 = 1.3266 \times 10^{-6}$
- 초기: $\xi(0) = [0,0,0.5]^T$ m, $\eta(0) = [0,0,0.3]^T$ rad

### 4개 시나리오

**Case (a): 외란 없는 비행**
- step 형태의 위치/yaw 기준 변경 (0–100s, 여러 시점에서 변경)
- Roll/Pitch 자동 생성 (≈ ±0.1 rad 범위에서 transient), control input 매끄러움. → 양호한 추적.

**Case (b): 불확실성 + 공력 외란**
- 모델 불확실성: $\Delta J = 0.5$ J·kg·m²
- 공력 힘 (10s, 30s, 40s 발생):
  $$\frac{1}{m} F_{ext} = [\sin(0.1t), \sin(0.1t), \sin(0.1t)]^T \text{ N}$$
- 공력 토크 (10s 발생):
  $$\Gamma_{ext} = [0.3\sin(0.1t), 0.3\sin(0.1t), 0.5\sin(0.1t)]^T \text{ N·m}$$
- LESO가 외란을 거부 → 목표 궤적 정확 추적. Roll/Pitch 범위 ±0.15 rad 정도까지 보상.

**Case (c): 센서 잡음 동반**
- 모든 센서에 white noise 추가, 외란도 유지.
- 추적은 성공하나 control signal에 erratic 성분 → 실험 구현 시 actuator 동역학과의 상호작용 고려 필요.

**Case (d): adaptive backstepping과 비교 ([5] Drouot et al. CEP 2014)**
- 3D 나선형 궤적 추적, 외란·불확실성 환경.
- **본 LESO 기반 제어가 더 빠른 수렴, 더 정확한 추적, 더 강한 강건성** 시연.

### 비교군
- PD/PID [16, 13], $H_\infty$ [4], adaptive backstepping [5] — 본 논문은 [5]와 직접 비교.

## COAX-SIM 연관

### 기존 COAX-SIM 구현 위치
- `C:\Users\jsp99\Work\COAX-SIM\Controller\LDOB.m`: 자세 채널 1차 Linear DOB.
  - 구조: `d_hat = z + L*J*omega`, `z_dot = -L*(d_hat + Torque_d)` (Euler 적분).
  - Transfer function: `d_hat/d = L/(s+L)` (1차 저역통과).
- `C:\Users\jsp99\Work\COAX-SIM\Controller\LDOB_pos.m`: translational(NED) 1차 Linear DOB, 위와 동일 구조.
- `C:\Users\jsp99\Work\COAX-SIM\Controller\BACKDOB.m`, `BACKHOCBFQPDOB.m`: backstepping/HOCBF에 1차 DOB 연결.

### 본 논문 LESO와의 차이
| 항목 | COAX-SIM LDOB | ISAT 2016 LESO |
|---|---|---|
| 차수 | 1차 ($d/(s+L)$) | 3차 (위치/속도/외란 동시 추정) |
| 추정 대상 | 외란 토크/힘만 | 위치 + 속도 + 외란 모두 |
| 이득 | scalar $L$ | $[\alpha_1/\epsilon, \alpha_2/\epsilon^2, \alpha_3/\epsilon^3]$ → 대역폭 $1/\epsilon$ |
| 안정성 증명 | 단일 채널 LP필터 | Lyapunov + bounded ball $B_r$ 지수수렴 정식 증명 |
| 외란 보상 폭 | bounded constant 외란에 강 | bounded time-varying derivative까지 보상 (sin 외란 보상 확인됨) |

### MATLAB 의사코드 (LESO 도입 시)
```matlab
function [zeta_hat, z] = LESO_pos(xi_meas, mu_xi, zeta_hat, p)
% 3차 Linear ESO — translational subsystem
% State: zeta_hat = [zeta1_hat; zeta2_hat; zeta3_hat] in R^9 (각 R^3)
%   zeta1 ≈ position xi
%   zeta2 ≈ velocity
%   zeta3 ≈ total disturbance (matched)
%
% Inputs:
%   xi_meas [3x1] : measured position (NED)
%   mu_xi   [3x1] : virtual control (commanded acceleration)
%   p       : struct with p.alpha = [a1; a2; a3], p.eps, p.dt_ctrl

    eps   = p.eps;
    a1    = p.alpha(1); a2 = p.alpha(2); a3 = p.alpha(3);

    err1  = xi_meas - zeta_hat(1:3);     % 출력 오차

    dz1 = zeta_hat(4:6)   + (a1/eps)    * err1;
    dz2 = mu_xi            + (a2/eps^2) * err1;
    dz3 = (a3/eps^3) * err1;

    zeta_hat = zeta_hat + [dz1; dz2; dz3] * p.dt_ctrl;
end

% 제어법칙 (eq. 23):
% mu_xi = xi_ddot_d - K_xi*(zeta_hat(1:3) - xi_d) ...
%               - K_v*(zeta_hat(4:6) - xi_dot_d) ...
%               - zeta_hat(7:9);
```
- `p.alpha`는 $A_0 - L_x C$의 desired pole(예: 다중극 $-1$) → Hurwitz Vandermonde 풀이.
- `p.eps` 작을수록 대역폭 증가(0.05~0.2 권장).

### 도입 시 이득점
1. **외란 보상 범위 확대**: 1차 LDOB가 잡지 못하는 시변(예: 풍속 sin 외란) 보상 가능.
2. **위치/속도 추정 통합**: 별도 칼만 필터 없이 LESO에서 동시 추정 → IMU 잡음에 더 강한 속도 추정.
3. **체계적 튜닝**: $\epsilon$ 한 개만으로 대역폭 조절. 기존 $L$ 트라이얼보다 직관적.
4. **자세 채널에도 동일 도입**: rotational LESO로 NDOB/LDOB 교체 가능, Lyapunov 보장.

### 도입 시 주의점
- LESO는 **peaking phenomenon**: $\epsilon$ 작을수록 초기 transient에서 추정치가 크게 튐 → 초기값 일치 또는 saturation 필요.
- 측정 잡음 직접 차분 → 잡음 증폭. low-pass + LESO 조합 권장 (논문 4.1에서도 2차 LP 필터로 reference 처리 언급).
- 본 논문은 quasi-stationary flight 가정 — aggressive maneuver는 future work로 명시.

## 핵심 수식 모음 (재현용)

1. **Newton–Euler 모델 (eq. 2)**:
$$m\ddot{\xi} = R_\eta T - m g z_e + F_{ext}$$
$$M(\eta)\dot{w} = \Psi_\eta^T \Gamma - C(\eta, \dot\eta) w + \Gamma_{ext}$$

2. **분리된 두 부시스템 (eq. 8)**:
$$\Sigma_T : \begin{cases} \dot\xi = v \\ m\dot v = T_z R_\eta^d z_e + T_z h(\eta_d, \delta_\eta) - mgz_e + F_{ext} \end{cases}$$
$$\Sigma_R : \begin{cases} \dot\eta = w \\ M(\eta)\dot w = \Psi_\eta^T \Gamma - C(\eta, w)w + \Gamma_{ext} \end{cases}$$

3. **선형화 변환 (eq. 11)**:
$$\ddot\xi = T_z R_\eta^d z_e + T_z h(\eta_d, \delta_\eta) - g z_e + \tilde F_{ext}$$
$$\dot w = \tilde\Gamma + \tilde\Gamma_{ext}$$

4. **추적 오차 동역학 (eq. 14)**:
$$\dot\delta_1 = A_1 \delta_1 + B_1[\mu_\xi - \ddot\xi_d + \tfrac{1}{m} T_z h + F_{ext}]$$
$$\dot\delta_2 = A_2 \delta_2 + B_2[\tilde\Gamma - \ddot\eta_d + \tilde\Gamma_{ext}]$$

5. **Translational LESO (eq. 21)**:
$$\dot{\hat\zeta} = A_0 \hat\zeta + B_0 \mu_\xi + LC(\zeta - \hat\zeta)$$

6. **Translational 제어 (eq. 23)**:
$$\mu_\xi = \ddot\xi_d - K_\xi(\hat\zeta_1 - \xi_d) - K_{v_\xi}(\hat\zeta_2 - \dot\xi_d) - \hat\zeta_3$$

7. **Rotational LESO (eq. 41)**:
$$\dot{\hat{\bar\zeta}} = A_0 \hat{\bar\zeta} + B_0 \tilde\Gamma + LC(\bar\zeta - \hat{\bar\zeta})$$

8. **Rotational 제어 (eq. 42)**:
$$\tilde\Gamma = \ddot\eta_d - K_\eta(\hat{\bar\zeta}_1 - \eta_d) - K_w(\hat{\bar\zeta}_2 - \dot\eta_d) - \hat{\bar\zeta}_3$$

9. **Lyapunov 부등식 (eq. 33, 51)**:
$$\dot V_0 \leq -\|\rho\|^2 \left[1 - \frac{2\epsilon \lambda_{\max}(P_0) h_{\max}}{\|\rho\|}\right]$$

10. **참 입력 복원 (eq. 7)**:
$$\Omega_1^2 = \frac{\gamma_2 T_z - \kappa_\alpha \tau_\psi}{\kappa_\alpha \gamma_2 + \kappa_\beta \gamma_1}, \quad \Omega_2^2 = \frac{\gamma_1 T_z - \kappa_\alpha \tau_\psi}{\kappa_\beta \gamma_1 + \kappa_\alpha \gamma_2}$$
$$\delta_{cx} = -\frac{\tau_\phi}{d \kappa_\beta \Omega_2^2}, \quad \delta_{cy} = \frac{\tau_\theta}{d \kappa_\beta \Omega_2^2}$$

11. **Closed-loop 합성 (eq. 31)**:
$$\begin{bmatrix} \dot\delta_1 \\ \dot\rho \end{bmatrix} = \begin{bmatrix} A_{1u} & [B_1 K_1 \ B_1] \epsilon^{3-i} \\ 0 & A_0 - L_x C \end{bmatrix} \begin{bmatrix} \delta_1 \\ \rho \end{bmatrix} + \begin{bmatrix} 0 \\ \epsilon E_0 \end{bmatrix} p(\zeta)$$

## 한계

### 저자 명시 한계
- **Quasi-stationary flight** 가정 → 고속/aggressive maneuver 미검증. Future work로 언급.
- Roll/Pitch 각도 제약 $|\phi|, |\theta| \leq \pi/2$ — singularity 회피 위한 모델 단순화.
- $\Sigma \approx 0$ 가정 (작은 swashplate 각) → 큰 cyclic 시 small body force 무시로 인한 오차 가능.
- ESO 잡음 민감성: 센서 잡음 시 control signal에 erratic 성분 (Case c) → 실제 actuator 동역학과 상호작용 우려.
- Disturbance 변화율 bounded 가정 — 외란 도함수가 시간 다항식(generalized ESO, [29] Madonski) 형태면 더 높은 차수 ESO 필요.

### COAX-SIM 적용 시 고려사항
- COAX-SIM은 정지비행/저속만 검증 시나리오 → 본 논문 가정과 일치, 즉시 이식 적합.
- 본 논문의 cyclic swashplate는 COAX-SIM 모델과 일치하는지 확인 필요 (`Plant/` 폴더의 추력/토크 매핑과 eq. 3,4 비교).
- 3차 LESO의 9D 상태 → MATLAB struct로 `eso_state` 변수 추가, `dt_ctrl` 적분.
- Peaking 회피: $\zeta_1(0) = \xi(0)$, $\zeta_2(0) = v(0)$, $\zeta_3(0) = 0$로 초기화.
- 본 논문은 NDOB(`NDOB.m`)를 COAX-SIM에서 찾지 못함 — `BACKDOB.m`/`LDOB.m`이 1차 DOB로 대응. LESO로 교체 시 의미있는 비교 실험 가능.
- `main.m` 시나리오에서 본 논문 Case (b)와 유사한 sin 외란 주입 → LESO vs LDOB 외란 추정 RMSE 비교가 자연스러운 검증 절차.
