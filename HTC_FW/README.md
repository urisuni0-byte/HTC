# HTC 펌웨어 (VSHCCB_V1, ATmega128A 16MHz)

PTC / Peltier 모드를 선택하고 프로토콜(PT1~PT5)을 실행합니다. 각 프로토콜은 6단계(온도, 시간)로 되어 있고, 단계마다 설정 온도를 설정 시간 동안 유지합니다. 1초마다 HMI로 그래프 데이터를 보냅니다.

## Microchip Studio
1. New Project → GCC C Executable Project → Device **ATmega128A**
2. 생성된 main.c를 이 `main.c`로 교체하고 `user_define.h`를 추가
3. Toolchain → AVR/GNU Linker → Libraries에 `libm`이 있는지 확인 (NTC 계산에 `log()` 사용)
4. **Fuse (필수)**
   - `M103C` 해제 (공장 출하 시 ATmega103 호환 모드가 켜져 있음)
   - `JTAGEN` 해제 (PF4/PF5 = FAN_ADC1/2를 ADC로 사용)
   - `BODEN` 켜기, BODLEVEL 4.0V
   - 외부 크리스탈 16MHz (CKSEL = Ext. Crystal/Resonator High Freq)

## 핀 맵 (회로도 VSHCCB_V1 기준)
| 기능 | 핀 | 동작 |
|---|---|---|
| PTC_OUT1 / PTC_OUT2 | PA6 / PA7 | **LOW = 히터 ON** (Q1 NPN에서 반전) |
| FAN_OUT1 (FAN_PTC, WJS 24V 0.1A) | PA3 | LOW = ON |
| FAN_OUT2 (FAN_PELT, 방열판) | PA4 | LOW = ON |
| HOT_ON/OFF → Q9A(P-ch) | PC4 | HIGH = ON |
| COOL_ON/OFF → Q10A(P-ch) | PC5 | HIGH = ON |
| HOT_PWM → Q12(N-ch) | PB5 / OC1A | 1kHz PWM |
| COOL_PWM → Q11(N-ch) | PE3 / OC3A | 1kHz PWM |
| PTC_S1 / PELT_S2 / SPARE_S3 (NTC 50K) | ADC0 / ADC1 / ADC2 | |
| FAN_ADC1 / FAN_ADC2 | ADC4 / ADC5 | FAN 전류 |
| LED D5 | PB0 | LOW = ON |
| TOUCH_COM (HMI) | USART1 (PD2/PD3) | 19200 8N1 |
| DEBUG_COM | USART0 (PE0/PE1) | 19200 8N1, 송신 데이터 복사 |

## HMI 통신 프로토콜 (ASCII, `:` 로 시작하고 CR LF로 끝남)
### HMI → MCU
| 명령 | 설명 | 응답 |
|---|---|---|
| `:MODE m` | 모드 선택 (0=없음, 1=PTC, 2=PELTIER) | `:OK` / `:FAIL` |
| `:SET m p s temp time` | 프로토콜 저장 (모드, PT 1~5, 단계 1~6, 온도 ℃, 시간 분 1~10, 0이면 그 단계 건너뜀). EEPROM에 저장 | `:OK` / `:FAIL` |
| `:GET m p` | 프로토콜 읽기 | `:PT m p t1 n1 t2 n2 … t6 n6` |
| `:RUN p` 또는 `:RUN m p` | 실행 | `:OK` / `:FAIL` |
| `:STOP` | 정지 | `:OK` |
| `:PAUSE` / `:RESUME` | 일시정지 / 재개 (재개하면 설정 온도에 다시 도달한 뒤 남은 시간을 셈) | `:OK` / `:FAIL` |
| `:CLR` | 에러 해제 (온도가 정상일 때만) | `:OK` / `:FAIL` |
| `:STATUS?` | 즉시 상태 요청 | `:DAT …` |

예) PTC PT1의 1단계를 95℃, 3분으로 설정: `:SET 1 1 1 95 3`

### MCU → HMI (자동 송신)
- **그래프 데이터, 1초마다**
  `:DAT mode state pt step sv pv remain hold out err elapsed`
  | 항목 | 내용 |
  |---|---|
  | mode | 0 / 1=PTC / 2=PELT |
  | state | 0=대기, 1=동작, 2=일시정지, 3=완료, 4=에러 |
  | pt, step | PT 번호, 단계 번호 (1~6, 화면의 "TEMP_1ST" 등) |
  | sv, pv | 설정 온도, 현재 온도. **단위 0.1℃** (예: 950 = 95.0℃) |
  | remain | 이 단계에 남은 시간(초). 화면의 `00:00` |
  | hold | 0=목표 온도로 가는 중, 1=유지 중 (유지 중일 때만 카운트다운) |
  | out | 출력 ‰ (PTC 0~1000, Peltier -1000 냉각 ~ +1000 가열) |
  | err | 에러 코드 (화면의 `ER: 0X`) |
  | elapsed | 실행 시작 후 경과 시간(초). 그래프 X축 |
- **완료**: `:END mode pt` → HMI가 부저 3회 울리고 데이터 저장
- **에러**: `:ERR n`

### 에러 코드
| 코드 | 내용 |
|---|---|
| 01 | PTC Heater 과열 (135℃ 이상) |
| 02 | Peltier 과열 (80℃ 이상) |
| 03 | FAN_Vial(PTC) FAN 전류 없음 |
| 04 | FAN_Peltier 전류 없음 |
| 05 | PTC Heater 단선 (출력 90% 이상으로 60초 동안 1℃도 오르지 않음) |
| 06 | Peltier 단선 (출력 90% 이상으로 120초 동안 1℃도 변하지 않음) |
| 07 / 08 | PTC / Peltier NTC 센서 단선·단락 (추가한 코드) |

## 펠티어 보호 초기값 (`user_define.h`)
| 항목 | 값 |
|---|---|
| 최대 duty | 냉각 70%, 가열 50% |
| 소프트 스타트 / 정지 | 100ms마다 1%씩 올림 / 2%씩 내림 |
| 극성 전환 | duty를 0까지 내리고 **5초** 쉰 뒤 반대 방향으로 켬 (열충격 방지) |
| 같은 레그 동시 ON | `Pelt_SetDuty()`에서 반대쪽 P-ch와 PWM을 항상 먼저 끔 (단락 방지) |
| 방열 FAN | 펠티어 동작 3초 전에 먼저 켜고, 정지 후 60초 더 돌림. FAN 이상이면 펠티어 즉시 정지 |
| 설정 범위 | 5~70℃, 80℃ 이상이면 차단 |
| 출력 데드밴드 | 3% 이하 출력은 끔 (극성이 자주 바뀌는 것 방지) |

## 확인 / 조정이 필요한 항목
1. **NTC 분압 저항**: R3/R4는 10K 3296 가변저항입니다. 실측값을 `NTC_R_PULLDOWN`에 넣으세요. B정수는 센서 사양으로 `NTC_BETA`(기본 3950)를 수정하세요.
2. **PID 값** (`PTC_KP/KI/KD`, `PELT_KP/KI/KD`)은 시작값입니다. 실제 장비에서 튜닝해야 합니다.
3. **HOT/COOL 방향**: 회로도 이름 기준으로 HEAT = Q9A+Q12 입니다. 펠티어 배선이 반대이면 냉각 명령에 온도가 오릅니다. 처음 시험할 때 확인하세요.
4. **FAN 전류 검출**: 4.7Ω × 48배이므로 0.1A FAN이면 ADC가 포화됩니다. 연결/단선만 판단하고, 회전 정지(Lock)는 감지하지 못합니다.
5. **PTC_USE_OUT2 = 1**: HT_OUT1과 HT_OUT2를 같이 구동합니다. 히터가 1개뿐이라도 문제없습니다.

## 하드웨어 주의
- **PTC 히터 회로는 MCU가 리셋 상태(핀 Hi-Z)일 때 켜집니다.** R13 10K가 Q1을 끄고, 그러면 Q2 게이트가 12V로 올라가기 때문입니다. 펌웨어는 시작하자마자 OFF를 출력하지만, 칩이 프로그램되지 않았거나 리셋이 반복되면 히터가 계속 켜집니다. PTC_OUT1/2에 5V 풀업(4.7K)을 추가하고, 히터 블록에 온도 퓨즈를 다는 것을 권장합니다.
- PBR50-24-12는 50W(12V 4.2A)입니다. 펠티어 정격 전류가 이보다 크면 PWM이 켜질 때 DC/DC가 과전류 보호에 걸릴 수 있습니다. 필요하면 `PELT_*_DUTY_MAX`를 낮추세요.
- SPARE_S3 NTC를 펠티어 방열판에 붙이고 `PELT_HS_SENSOR_USE = 1`로 하면, 방열판이 60℃ 이상일 때 차단합니다.
