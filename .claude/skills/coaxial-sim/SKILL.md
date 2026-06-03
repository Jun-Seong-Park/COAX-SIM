---
name: coaxial-sim
description: >
  동축반전로터 드론 검증 시뮬레이션 설정·실행 전문 스킬.
  main.m에 하드코딩 파라미터를 설정하고, 위치 박스 테스트(x/y/z ±1m 순차)
  및 자세 스트레스 테스트(phi/theta/psi ±15° 순차)를 세 제어기
  (BSC+DOB, CBF+DOB+BSC, MPC)에 대해 검증하는 절차를 안내한다.
  "검증 시뮬", "validation run", "main.m 수정", "제어기 비교", "시나리오 하드코딩",
  "위치 박스", "자세 테스트" 키워드가 포함된 요청에 적극적으로 사용한다.
---

# Coaxial-Sim — 검증 시뮬레이션 실행 가이드

> **역할:** `main.m`을 수정·구성하여 표준화된 검증 시뮬을 실행하는 방법을 안내한다.
> 기존 루프 아키텍처는 **절대 변경하지 않는다.**

---

## 0. 보존해야 할 루프 아키텍처

```
scenario_fn(t)  →  X_d [x; y; z; yaw_d]
    ↓
POS2THR(X[1:3], X_d[1:3], error_prev, angle_max)  →  ACC_d, Acc_Max, THR_d
    ↓
POS2ATT(X[1:3], X_d[1:3], ACC_d, THR_d, 1)  →  X_d[4:6]   ← 자세 테스트 시 우회
    ↓
NDOB(X_1dot, torque_d, 1)  →  d_hat
    ↓
Desired(4:6,:) = [X_d(4:6), X_d_dot, X_d_ddot]   (수치 미분)
    ↓
[제어기 — controller_mode 로 선택]
    ↓
compute_rotor_speed_CoaX  →  X_d[13:16]
    ↓
CoaX_Dyn4  →  Force, Moment  →  sixdof  →  X 갱신
```

**변경 금지:** 루프 내 변수명, `savevec`·`sixdof`·`CoaX_Dyn4`·`compute_rotor_speed_CoaX` 호출 순서,
전역 시간 변수(`dt_sim`, `dt_ctrl`, `dt_save`, `dt_traj`).

---

## 1. 검증에서 달라지는 것 — 한눈에 보기

| 항목 | 기존 main.m | 검증 설정 |
|------|-------------|-----------|
| 시뮬 시간 | `tf = 10` | `tf = 90` (위치 박스) / `tf = 30` (자세) |
| 시나리오 | Scenario1~9 | `Scenario_validate` (신규) / `Scenario0` (자세) |
| 제어기 선택 | BACKHOCBFQP 줄만 활성 | `controller_mode` 변수로 분기 |
| 자세 직접 명령 | 없음 | `validate_attitude_mode` + `attitude_ref` |
| MPC 초기화 | 주석 처리 | `controller_mode = 'MPC'` 시 조건부 실행 |
| 실행 간 상태 초기화 | 없음 | `clear NDOB` 매 실행 전 필수 |
| 결과 저장 | 없음 | `.mat` 파일 저장 후 비교 |

---

## 2. Scenario_validate.m — 위치 박스 순차 시나리오

`Scenario/Scenario_validate.m` 파일을 새로 만든다.

```matlab
function X_d = Scenario_validate(t)
%% Scenario_validate — 위치 박스 순차 테스트
% NED 좌표계: z 아래가 양수. 고도 1m = z=-1, 고도 2m = z=-2.
%
% 구간별 목표 위치 [x; y; z; yaw_d]:
%  [  0, 10) : 호버 (0, 0, -1)  — 기준선
%  [ 10, 20) : x=+1m
%  [ 20, 30) : x=-1m
%  [ 30, 40) : y=+1m
%  [ 40, 50) : y=-1m
%  [ 50, 65) : z=-2m (고도 2m, 상승 +1m)
%  [ 65, 80) : z= 0m (지면 수준, 하강 -1m)
%  [ 80, inf): 복귀 호버 (0, 0, -1)

    z_base = -1;   % 기준 고도 (NED, 고도 1m)

    if     t <  10,  X_d = [ 0;  0;  z_base; 0];
    elseif t <  20,  X_d = [ 1;  0;  z_base; 0];   % x+1
    elseif t <  30,  X_d = [-1;  0;  z_base; 0];   % x-1
    elseif t <  40,  X_d = [ 0;  1;  z_base; 0];   % y+1
    elseif t <  50,  X_d = [ 0; -1;  z_base; 0];   % y-1
    elseif t <  65,  X_d = [ 0;  0;  -2;     0];   % z=-2 (고도 2m)
    elseif t <  80,  X_d = [ 0;  0;   0;     0];   % z=0  (지면)
    else,            X_d = [ 0;  0;  z_base; 0];   % 복귀
    end
end
```

> `tf = 90` 이상으로 설정해야 전 구간을 커버한다.

---

## 3. main.m — 검증 설정 블록 삽입

`%% Save Slot` 섹션 **바로 위**에 다음 블록을 통째로 삽입한다.
기존 파라미터 줄은 그대로 두고 이 블록이 필요한 값만 덮어쓴다.

```matlab
%% ============================================================
%% 검증 설정 블록 (Validation Configuration)  ← 이 블록만 편집
%% ============================================================

% --- 시뮬레이션 시간 재설정 ---
tf = 90;   % 위치 박스 전 구간 (자세 테스트 단독: tf = 30)

% tf 변경 시 시간 벡터 재계산 (필수)
t_sim      = t0:dt_sim:tf;
t_ctrl     = t0:dt_ctrl:tf;
n_sim      = length(t_sim);
n_ctrl     = length(t_ctrl);
ctrl_steps = round(dt_ctrl / dt_sim);
save_steps = round(dt_save / dt_sim);

% --- 제어기 선택 ---
% 'BACKDOB'      : BSC + DOB  (barrier 없음)
% 'BACKHOCBFQP'  : CBF + DOB + BSC  (기본 검증 제어기)
% 'MPC'          : Model Predictive Control
controller_mode = 'BACKHOCBFQP';

% --- 자세 직접 명령 모드 ---
% false: 정상 (POS2ATT가 X_d(4:6) 계산)
% true : POS2ATT 우회, attitude_ref 를 X_d(4:6)으로 직접 설정
validate_attitude_mode = false;

% --- 직접 자세 명령 (validate_attitude_mode = true 시 유효) ---
% 의도적으로 ±90° 입력 → CBF가 ±2° 이내로 클램핑하는지 스트레스 확인
attitude_ref = [deg2rad(90); 0; 0];   % [phi_d; theta_d; psi_d]

% --- 시나리오 함수 선택 ---
scenario_fn = @Scenario_validate;   % 위치 박스 테스트
% 자세 테스트 시: scenario_fn = @Scenario0;  (위치 고정 호버)

% --- MPC 초기화 (MPC 모드 시만 실행) ---
if strcmp(controller_mode, 'MPC')
    fprintf('MPC 초기화 중...\n');
    [x_mpc, mpcobj] = mpcset(state_max);
    fprintf('MPC 초기화 완료.\n');
end

% --- persistent 상태 초기화 (실행 간 NDOB 오염 방지) ---
% 주의: 루프 시작 전 1회만. 루프 내 호출 금지.
clear NDOB
```

---

## 4. main.m — 루프 내 제어기 분기 교체

루프 내 `%% Attitude Controllers` 섹션을 아래로 **통째로 교체**한다.
기존 주석 줄들(`% torque_d = BACKDOB...`, `% torque_d = MPCctrl...`)은 삭제해도 된다.

```matlab
        %% Attitude Controllers — controller_mode 분기
        switch controller_mode
            case 'BACKDOB'
                torque_d = BACKDOB(X(4:6,:), X(10:12,:), Desired(4:6,:), ...
                                   Torque_max, d_hat);

            case 'BACKHOCBFQP'
                [torque_d, torque_nom, torque_limit] = BACKHOCBFQP( ...
                    X(4:6,:), X(10:12,:), Desired(4:6,:), ...
                    state_max, P1, P2, P3, torque_filtered, d_hat);
                torque_filtered = torque_d;

            case 'MPC'
                torque_d = MPCctrl(X(4:6,:), X(10:12,:), Desired(4:6,:), ...
                                   mpcobj, x_mpc);

            otherwise
                error('알 수 없는 controller_mode: %s', controller_mode);
        end
```

> **BACKHOCBFQP 함수 서명 주의:**
> `BACKHOCBFQP.m` 선언은 8개 인수이지만 main.m은 9번째로 `d_hat`을 전달한다.
> MATLAB이 초과 인수를 무시하므로 실행 오류는 없으나, **현재 DOB 보상이 실제로 미적용 상태다.**
> 재검증 목적으로는 현재 상태로 실행하고, DOB 효과를 실제로 확인하려면
> `BACKHOCBFQP.m` 함수 선언에 `d_hat` 인수를 추가하고 내부에서 보상 처리가 필요하다.

---

## 5. main.m — validate_attitude_mode 패치

루프 내 `%% Guidance` 블록 바로 뒤(NDOB 호출 전)에 3줄을 추가한다.

```matlab
        %% Guidance
        [ACC_d, Acc_Max, THR_d, error_prev(1:3,:)] = POS2THR( ...
            X(1:3,:), X_d(1:3,:), error_prev(1:3,:), angle_max);
        X_d(4:6,:) = POS2ATT(X(1:3,:), X_d(1:3,:), ACC_d, THR_d, 1);
        X_d(6,:)   = yaw_d;

        % ↓ 자세 직접 명령 패치 — POS2ATT 결과를 attitude_ref로 덮어쓰기
        if validate_attitude_mode
            X_d(4:6,:) = attitude_ref;   % [phi_d; theta_d; psi_d]
        end
```

> `BACKHOCBFQP` 사용 시 QP가 ±2° 이내로 클램핑하므로 실제 자세는 제약을 넘지 않는다.
> `BACKDOB`는 saturation만 적용, `MPC`는 자체 제약 적용.

---

## 6. 위치 박스 테스트 실행 절차

### 단일 제어기 실행 (권장 방법)

`main.m` 검증 설정 블록에서 값을 변경하고 **파일을 저장한 후** 실행:

```matlab
tf                     = 90;
controller_mode        = 'BACKHOCBFQP';   % 'BACKDOB' / 'MPC' 로 변경
validate_attitude_mode = false;
scenario_fn            = @Scenario_validate;
```

MATLAB 커맨드 창:

```matlab
clear NDOB       % ← 매 실행 전 필수 (persistent z 초기화)
run('main.m')
```

> `main.m` 첫 줄이 `clc; clear all`이므로 workspace 변수 주입은 불가능하다.
> 반드시 **파일을 직접 수정하고 저장한 뒤 실행**해야 한다.

### 세 제어기 순차 배치

```
1. controller_mode = 'BACKDOB'      → 저장 → clear NDOB → run('main.m')
   결과 저장: save('result_pos_BACKDOB.mat', 'X_s','X_d_s','torque_d_s','B1_s','B2_s','t_save')

2. controller_mode = 'BACKHOCBFQP'  → 저장 → clear NDOB → run('main.m')
   결과 저장: save('result_pos_BACKHOCBFQP.mat', ...)
   (BACKHOCBFQP만 torque_limit_s 존재:)
   if exist('torque_limit_s','var'), save('result_pos_BACKHOCBFQP.mat','torque_limit_s','-append'); end

3. controller_mode = 'MPC'          → 저장 → clear NDOB → run('main.m')
   결과 저장: save('result_pos_MPC.mat', ...)
```

---

## 7. 자세 스트레스 테스트 실행 절차

### 테스트케이스 표

| 케이스 | `attitude_ref` | 테스트 목적 |
|--------|----------------|-------------|
| ATT-1 | `[deg2rad( 90);  0;  0]` | phi +90° → CBF ±2° 클램핑 확인 |
| ATT-2 | `[deg2rad(-90);  0;  0]` | phi -90° |
| ATT-3 | `[0;  deg2rad( 90);  0]` | theta +90° |
| ATT-4 | `[0;  deg2rad(-90);  0]` | theta -90° |
| ATT-5 | `[0;  0;   deg2rad( 90)]` | psi +90° (yaw — CBF 적용 여부 실험적 확인) |
| ATT-6 | `[0;  0;   deg2rad(-90)]` | psi -90° |

### 자세 테스트 설정

```matlab
tf                     = 30;
controller_mode        = 'BACKHOCBFQP';
validate_attitude_mode = true;
attitude_ref           = [deg2rad(90); 0; 0];   % 케이스별 변경
scenario_fn            = @Scenario0;            % 위치 고정 호버
```

케이스별로 `attitude_ref`를 교체하고 → 저장 → `clear NDOB` → `run('main.m')` 반복.
결과 저장 파일명 예: `result_att_phi_pos90.mat`, `result_att_theta_neg90.mat` 등.

---

## 8. MPC 초기화 상세

```matlab
% 검증 설정 블록 내: state_max 계산 이후에 위치 필수
if strcmp(controller_mode, 'MPC')
    [x_mpc, mpcobj] = mpcset(state_max);
end
```

- `mpcset.m`은 전역 변수 `Ixx`, `Iyy`, `Izz`, `dt_ctrl`을 사용 → main.m 파라미터 블록 이후에 호출
- `state_max(:,3)` (Torque_max)는 루프 내 매 스텝 갱신되지만 MPC 제약은 초기화 시점에 고정
- 초기화가 느리면 캐싱:

```matlab
% 최초 실행 후:
save('mpcobj_cache.mat', 'mpcobj');
% 재실행 시 (state_max 미변경 조건):
load('mpcobj_cache.mat');
x_mpc = mpcstate(mpcobj);   % 상태는 항상 새로 초기화
```

---

## 9. Persistent 상태 초기화 규칙

```
clear all       → workspace 변수만 삭제. persistent 변수 유지됨.  ← main.m 첫 줄이 이걸 함
clear NDOB      → NDOB의 persistent z 변수 초기화.  ← 올바른 방법
clear functions → 모든 함수의 persistent 변수 초기화 (핵 버튼)
```

**올바른 실행 순서 (매 시뮬 전):**

```matlab
clear NDOB      % (1) 관측기 상태 리셋
run('main.m')   % (2) main.m 실행 (savevec 자동 초기화 포함)
```

---

## 10. 결과 비교 분석 스크립트

### Barrier 위반 합계 (논문 Table 재현)

```matlab
labels = {'BACKDOB', 'BACKHOCBFQP', 'MPC'};
angle_names = {'phi', 'theta', 'psi'};
fprintf('%-15s | B1_phi   B1_theta  B1_psi  | B2_phi   B2_theta  B2_psi\n', 'Controller');
for k = 1:length(labels)
    d = load(sprintf('result_pos_%s.mat', labels{k}), 'B1_s', 'B2_s');
    v1 = arrayfun(@(i) sum(d.B1_s(i, d.B1_s(i,:)<0)), 1:3);
    v2 = arrayfun(@(i) sum(d.B2_s(i, d.B2_s(i,:)<0)), 1:3);
    fprintf('%-15s | %8.4f %8.4f %8.4f | %8.4f %8.4f %8.4f\n', labels{k}, v1, v2);
end
```

### 위치 추종 RMS 오차

```matlab
for k = 1:length(labels)
    d   = load(sprintf('result_pos_%s.mat', labels{k}), 'X_s', 'X_d_s');
    err = d.X_s(1:3,:) - d.X_d_s(1:3,:);
    rms = sqrt(mean(err.^2, 2));
    fprintf('%s — x:%.4f  y:%.4f  z:%.4f [m RMS]\n', labels{k}, rms(1), rms(2), rms(3));
end
```

### 자세 테스트 — CBF 클램핑 확인

```matlab
att_cases = {'phi_pos90','phi_neg90','theta_pos90','theta_neg90','psi_pos90','psi_neg90'};
idx_map   = [4, 4, 5, 5, 6, 6];
for k = 1:length(att_cases)
    d = load(sprintf('result_att_%s.mat', att_cases{k}), 'X_s');
    max_deg = max(abs(rad2deg(d.X_s(idx_map(k), :))));
    pass    = max_deg <= 2.01;   % angle_max = 2° + 허용오차
    fprintf('%-15s : 최대 %.4f deg  → %s\n', att_cases{k}, max_deg, string(pass));
end
```

---

## 11. 실행 전 체크리스트

- [ ] `Scenario/Scenario_validate.m` 존재 확인
- [ ] main.m `%% 검증 설정 블록` 삽입 및 **저장** 확인
- [ ] `tf` 재계산 코드(t_sim, n_sim 등)가 `tf = ...` 바로 아래에 있는지 확인
- [ ] `controller_mode` 값이 `'BACKDOB'` / `'BACKHOCBFQP'` / `'MPC'` 중 하나
- [ ] `validate_attitude_mode` 플래그 확인 (위치 박스: `false`, 자세 테스트: `true`)
- [ ] MPC 시: `mpcset` 호출이 `state_max` 계산 이후에 위치하는지 확인
- [ ] `clear NDOB` → `run('main.m')` 순서 지키기
- [ ] 결과 `.mat` 파일명 중복 없는지 확인

---

## 12. 자주 발생하는 오류

### QP Infeasible — `quadprog exitflag: -2`

```matlab
% 원인: 초기 상태가 barrier 외부 또는 P1/P2 너무 큼
disp(rad2deg(X(4:6)))   % 현재 자세 확인
P1 = 50;   % 100 → 50 으로 줄이기 (검증 설정 블록에 추가)
```

### NDOB d_hat 발산

```matlab
d_hat = NDOB(X_1dot, torque_d, 0.5);   % L: 1 → 0.5
```

### NED 고도 혼동 (드론이 아래로 떨어짐)

```matlab
% 고도 1m 상승 명령:
X_d(3) = -1;   % 올바름 (NED, 음수 = 위)
X_d(3) =  1;   % 잘못됨 (지면 아래)
```

### MPC 실시간 연산 초과

MPC는 `dt_ctrl = 0.0025s`(400Hz) 내 계산이 어려울 수 있다 (논문에서도 언급된 한계).

```matlab
% mpcset.m 내부 지평선 단축:
mpcobj.PredictionHorizon = 20;   % 40 → 20
mpcobj.ControlHorizon    = 4;    % 8 → 4
```

### `torque_limit_s` 변수 없음 오류

`BACKDOB`와 `MPCctrl`은 `torque_limit`을 반환하지 않는다.
main.m 저장 블록의 `if exist('torque_limit', 'var')` 가드가 처리하므로 저장 시 오류 없음.
비교 분석 시:

```matlab
if exist('torque_limit_s', 'var')
    % BACKHOCBFQP 결과에서만 사용 가능
end
```

---

## 13. 하드코딩 파라미터 기준값 (수정 금지)

```matlab
% 시간 (변경 금지)
dt_sim  = 0.0001;   % 10 kHz
dt_ctrl = 0.0025;   % 400 Hz
dt_traj = 0.01;     % 100 Hz
dt_save = 0.1;      % 10 Hz

% 안전 제약 (CBF 기준, 변경 금지)
angle_max     = [deg2rad(2); deg2rad(2); deg2rad(2)];   % ±2°
angle_dot_max = [deg2rad(5); deg2rad(5); deg2rad(5)];   % ±5°/s

% CBF 파라미터 (검증 기준값)
P1 = 100; P2 = 100; P3 = 100;   % 스칼라, 함수 내에서 eye(3) 곱해짐

% 물리 파라미터 (절대 변경 금지)
Mass = 11.5;  Ixx = 0.2203;  Iyy = 0.2567;  Izz = 0.1056;
```

---

## 14. 관련 파일 위치

```
시뮬/
├── main.m                              ← 수정 대상 (검증 설정 블록 삽입)
├── Scenario/
│   ├── Scenario0.m                    ← 호버 시나리오 (자세 테스트용)
│   └── Scenario_validate.m            ← 신규 작성 (위치 박스 테스트)
├── Controller/
│   ├── BACKDOB.m                      ← BSC+DOB  (5인수: state, state_1dot, Desired, Torque_max, d_hat)
│   ├── BACKHOCBFQP.m                  ← CBF+DOB+BSC (선언 8인수, 9번째 d_hat 전달되나 미사용)
│   ├── MPCctrl.m                      ← MPC  (5인수: state, state_1dot, Desired, mpcobj, x_mpc)
│   ├── mpcset.m                       ← MPC 초기화  (1인수: state_max)
│   └── NDOB.m                         ← 외란 관측기  (persistent z → clear NDOB 필수)
├── Trajectory/
│   ├── POS2THR.m                      ← 위치 → 가속도/추력 변환
│   └── POS2ATT.m                      ← 가속도 → 자세 변환 (validate_attitude_mode 시 우회)
└── .claude/skills/
    ├── coaxial-master/skill.md        ← 도메인 지식 (이론·수식·파라미터 상세)
    └── coaxial-sim/SKILL.md           ← 이 파일 (검증 실행 절차)
```
