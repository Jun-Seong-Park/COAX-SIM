# Coaxial Rotor Drone — Knowledge Base

COAX-SIM 프로젝트의 참고 논문(`../references/`) 요약과 공통 동역학 지식을 정리한 폴더이다.
각 .md 파일은 단독으로 읽어도 의미가 통하도록 작성되었으나, **새로운 작업자는 `00-common-coaxial-dynamics.md`를 먼저 읽고 개별 논문 요약으로 넘어가는 것을 권장**한다.

---

## 파일 구성

### 0. 공통 지식 (먼저 읽을 것)

| 파일 | 내용 |
|------|------|
| [00-common-coaxial-dynamics.md](00-common-coaxial-dynamics.md) | 동축반전 드론의 좌표계, 6DOF 동역학, 공력 모델, HOCBF/Backstepping 표준 형태, 액추에이터 동역학, 외란/관측기, 계층 제어 구조, COAX-SIM 핵심 파라미터·디버깅 체크리스트 |

### 1. 중심 논문 (Park & Kim, COAX-SIM의 직접 기반)

| 파일 | 출처 | 핵심 |
|------|------|------|
| [park-kim-hocbf-coaxial-attitude.md](park-kim-hocbf-coaxial-attitude.md) | 박준성·김형근, 인천대 (KSME?) | HOCBF + Backstepping + QP로 동축반전 자세 안전제어. MPC 대비 계산시간 1/10. `BACKHOCBFQP.m`의 줄별 매핑 포함 |

### 2. 동역학 / 시스템 식별 (Schafroth muFly 시리즈)

| 파일 | 출처 | 핵심 |
|------|------|------|
| [schafroth-2010-cep-robust-control.md](schafroth-2010-cep-robust-control.md) | Schafroth 외, *Control Engineering Practice* 2010 | muFly(95 g) 6DOF 비선형 + CMA-ES 식별 + dual 2-dof H∞ 강인제어 (γ=1.38). COAX-SIM의 `CoaX_Dyn.m`/`compute_rotor_speed_CoaX.m` 출처 |
| [schafroth-2010-jirs-modeling.md](schafroth-2010-jirs-modeling.md) | Schafroth 외, *Journal of Intelligent & Robotic Systems* 2010 | 18-state 모델, stabilizer bar/swash/BLDC 1차계, Table 1 식별 파라미터. CEP 논문의 모델링·식별 쪽 본문 |

### 3. 외란 관측 / 강건 제어 (ISA Transactions 시리즈)

| 파일 | 출처 | 핵심 |
|------|------|------|
| [isat-2016-eso-coaxial-uav.md](isat-2016-eso-coaxial-uav.md) | *ISA Transactions* 2016 | 3차 LESO 기반 외란/불확실성 추정 + ADRC. COAX-SIM의 1차 LDOB와 비교, peaking 주의 및 도입 의사코드 포함 |
| [isat-2017-dob-hierarchical-coaxial.md](isat-2017-dob-hierarchical-coaxial.md) | *ISA Transactions* 2017 | DOB 기반 외측(위치) / 내측(자세) 계층 제어. `POS2THR.m`, `ACC2ATT.m`, `BACKDOB.m`, `LDOB*.m` 의 직접 매핑 |
| [isat-2023-tde-fuzzy-glida.md](isat-2023-tde-fuzzy-glida.md) | H. E. Glida 외, *ISA Transactions* 2023 | 모델프리 PID + TDE + 적응 퍼지 + FPA 오프라인 게인 튜닝 (OMFFC). COAX-SIM에 TDE를 NDOB 대안으로 도입 시 검토 자료 |

### 4. Backstepping 일반론

| 파일 | 출처 | 핵심 |
|------|------|------|
| [cep-2014-hierarchical-backstepping-mav.md](cep-2014-hierarchical-backstepping-mav.md) | Drouot 외, *Control Engineering Practice* 2014 (Gun Launched MAV) | 4단계 backstepping (δ₁→δ₂→ε₁→ε₂) 유도, Lyapunov, 적응 식별자, T_z=0 특이점. COAX-SIM의 backstepping 게인 설계·안정성 정당화 근거 |

---

## 사용법

### 새 제어기/관측기를 설계할 때
1. `00-common-coaxial-dynamics.md` §13(HOCBF) 또는 §12(Backstepping) 표준 식 확인
2. 관련 논문 요약의 **"COAX-SIM 연관"** 절 읽기
3. 논문의 핵심 수식 → MATLAB 의사코드 매핑 그대로 활용

### 파라미터를 튜닝할 때
1. `00-common-coaxial-dynamics.md` §14 (기본값) 확인
2. `00-common-coaxial-dynamics.md` §17 (디버깅 체크리스트) 확인
3. 해당 기능의 출처 논문 (예: HOCBF P1/P2/P3 → park-kim 논문) 참조

### 새 논문을 본 폴더에 추가할 때
1. PDF를 `../references/`에 배치
2. 본 폴더에 `<topic>-<year>-<short-title>.md` 형식으로 요약 작성
3. 본 README의 "파일 구성" 표에 한 줄 추가
4. `00-common-coaxial-dynamics.md` §20(매핑 표)도 필요 시 갱신

---

## 작성 원칙

- **한국어 서술, 영문 수식/용어 그대로**: 코드와 논문을 양쪽 모두 참조하기 쉽게
- **추측 금지**: PDF에 없는 내용은 적지 않는다
- **코드 매핑 우선**: 각 논문 요약에는 반드시 COAX-SIM의 어느 .m 파일에 해당하는지 표시
- **수식 출처 명시**: 가능하면 논문 식 번호 (Eq.X)를 그대로 인용
- **길이는 밀도로**: 분량을 채우려고 반복하지 않는다

---

## 코드 ↔ 논문 빠른 매핑

| MATLAB 함수 | 주요 논문 |
|-------------|----------|
| `Dynamics/sixdof.m` | (일반 강체) |
| `Dynamics/CoaX_Dyn.m` | Schafroth 2010 CEP/JIRS |
| `Dynamics/RotorDynSchafroth.m` | Schafroth 2010 CEP |
| `Dynamics/CoaX_ActDyn.m`, `ServoActDyn.m`, `RotorActDyn.m` | Schafroth 2010 CEP (1차 lag 부분) |
| `Controller/compute_rotor_speed_CoaX.m` | Schafroth 2010 CEP (control allocation) |
| `Controller/POS2THR.m` | ISAT 2017 (Eq.31, outer loop) |
| `Controller/ACC2ATT.m` | ISAT 2017 (Eq.34, virtual control inversion) |
| `Controller/BACKDOB.m` | CEP 2014 (backstepping) + ISAT 2017 (DOB 결합) |
| `Controller/BACKHOCBFQP.m` | **Park & Kim** (중심 논문) |
| `Controller/BACKHOCBFQPDOB.m` | Park & Kim + ISAT 2017 |
| `Controller/LDOB.m` | ISAT 2017 (1차 변형) — cf. ISAT 2016 LESO |
| `Controller/LDOB_pos.m` | ISAT 2017 (위치) — cf. ISAT 2016 LESO |
| `Controller/MPCctrl.m` | (MATLAB MPC Toolbox baseline, 비교군) |

---

## 향후 추가 후보 (이 폴더에 들어올 만한 주제)

- C3BF / DPCBF (collision-cone CBF) — 위치-level 안전 제약 확장
- DOCBF — 다중 동적 장애물 회피
- L1 adaptive control — TDE 대안
- MPC + CBF 통합 — Cascade CBF-MPC, RG-CBF
- INDI (Incremental Nonlinear Dynamic Inversion) — Schafroth 모델 의존도 ↓

각 후보의 PDF가 `../references/`에 추가될 때 개별 .md 작성.
