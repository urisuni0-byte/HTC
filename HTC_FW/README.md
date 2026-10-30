# HTC 펌웨어 (VSHCCB_V1, ATmega128A 16MHz)

PTC / Peltier 모드를 선택하고 프로토콜(PT1~PT5)을 실행합니다. 각 프로토콜은 6단계(온도, 시간)로 되어 있고, 단계마다 설정 온도를 설정 시간 동안 유지합니다. HMI가 500ms마다 상태와 온도를 읽어 가서 그래프를 그립니다.

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
| TOUCH_COM (HMI) | USART1 (PD2/PD3) | Modbus RTU 9600 8N1 |
| DEBUG_COM | USART0 (PE0/PE1) | 19200 8N1, 1초 상태 출력 |

## HMI 통신 (Modbus RTU, 문서 「VSHCCB RS232 통신 프로토콜」 기준)
- USART1(TOUCH_COM), **9600 8N1**, 슬레이브 주소 1, FC 03 / 06 / 16
- 프레임 끝 판단: 5ms 동안 수신이 없으면 프레임 끝 (3.5문자 = 3.65ms)
- 응답은 인터럽트로 송신합니다. SAVE의 EEPROM 저장도 한 바이트씩 나눠서 하므로 응답이 늦어지지 않습니다 (100ms 이내).
- CRC 에러, 다른 주소의 요청에는 응답하지 않습니다. 잘못된 주소나 기능 코드에는 예외 응답(01/02/03)을 보냅니다.

| 주소 | 내용 | 디바이스 처리 |
|---|---|---|
| 0 MODE | 1=PTC, 2=PELTIER | RUN할 때 읽음 |
| 1 PT_NO | 1~5 | RUN할 때 읽음 |
| 2 CMD | 1 RUN, 2 PAUSE, 3 SAVE, 4 RESUME, 5 STOP | 쓰기가 들어올 때마다 처리하고 0으로 되돌림 |
| 3 SAVE_PT | 1~5 | SAVE할 때 읽음 |
| 10~21 | 단계별 온도×10 / 시간(분) | RUN할 때 운전 테이블로 복사 |
| 100 STATE | 0 대기, 1 운전, 2 완료, 3 에러, 4 일시정지 | |
| 101 STEP | 1~6 | |
| 102 ELAPSED | RUN 이후 경과 초 (일시정지 중에는 멈춤) | |
| 103 PT_TEMP | 현재 온도×10 (센서 이상이면 0) | 대기 중에는 MODE 레지스터의 센서 값 |
| 104 ERROR | 0~6 | |

명령 처리 규칙
- **RUN**: 레지스터 10~21의 값(20.0~120.0℃, 0~10분)을 검사해서, 범위를 벗어나면 무시합니다. 펠티어 모드에서는 70.0℃가 넘는 설정을 70.0℃로 제한합니다. 운전 중에 들어온 RUN은 무시합니다.
- **PAUSE (2)**: 운전 중일 때만 일시정지합니다. 출력을 끄고 STEP과 ELAPSED는 그대로 둡니다 (STATE=4).
- **STOP (5)**: 운전, 일시정지, 완료 상태에서 운전을 끝내고 대기(STATE=0)로 돌아갑니다. 출력은 모두 끄고, 펠티어 방열 팬만 60초 더 돌립니다. STEP과 ELAPSED는 다음 RUN 때 초기화됩니다. 에러 상태에서는 STOP을 받아도 에러를 유지합니다.
- STOP 코드 5는 문서에 없어서 임의로 정한 값입니다. HMI의 실제 값이 다르면 `user_define.h`의 `CMD_STOP`만 바꾸면 됩니다.
- **RESUME**: 멈췄던 단계부터 이어서 운전합니다. 설정 온도에 다시 도달한 뒤 남은 시간을 셉니다.
- **SAVE**: 현재 MODE의 SAVE_PT번 PT로 EEPROM에 저장합니다. 운전에는 RUN할 때 화면이 보내는 값을 씁니다.
- 각 단계는 설정 온도 ±1℃에 도달한 뒤부터 단계 시간을 셉니다. 6단계가 끝나면 STATE=2(완료)가 됩니다.

DEBUG_COM(USART0, 19200)으로는 1초마다 `:DAT mode state pt step sv pv remain hold out err elapsed`를 출력합니다 (시험용).

### 에러 코드 (ERROR 레지스터)
| 코드 | 내용 | 해제 |
|---|---|---|
| 01 | PTC Heater 과열 (135℃ 이상) | 125℃ 미만으로 5초 지나면 자동 해제 |
| 02 | Peltier 과열 (80℃ 이상) | 70℃ 미만으로 5초 지나면 자동 해제 |
| 03 | FAN_Vial(PTC) FAN 전류 없음 | 다음 RUN 명령에서 해제 후 다시 확인 |
| 04 | FAN_Peltier 전류 없음 | 다음 RUN 명령에서 해제 |
| 05 | PTC Heater 단선 (출력 90% 이상으로 60초 동안 1℃도 오르지 않음) 또는 PTC 센서 단선·단락 | 히터: 다음 RUN / 센서: 정상으로 5초 지나면 자동 |
| 06 | Peltier 단선 (120초 동안 1℃도 변하지 않음) 또는 Peltier 센서 단선·단락 | 위와 같음 |

에러가 해제되면 ERROR=0, STATE=0(대기)이 됩니다.

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

## 미지원
- 운전 데이터 로그 저장은 지원하지 않습니다. DEBUG_COM의 1초 상태 출력은 시험용입니다.
