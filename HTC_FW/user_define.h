
/*
 * user_define.h : HTC (PTC / Peltier 온도 제어) - ATmega128A 16MHz
 */
//261030

#ifndef USER_DEFINE_H_
#define USER_DEFINE_H_

//================================================================
// 모드 / 상태
//================================================================
#define			MODE_NONE			0
#define			MODE_PTC			1
#define			MODE_PELT			2

#define			ST_IDLE				0
#define			ST_RUN				1
#define			ST_PAUSE			2
#define			ST_END				3
#define			ST_ERROR			4

#define			PT_MAX				5		//프로토콜 PT1~PT5
#define			STEP_MAX			6		//프로토콜당 6단계

//================================================================
// 에러 코드 (HMI "ER: 0X")
//================================================================
#define			ERR_NONE			0
#define			ERR_PTC_OVER		1		//ER01 PTC Heater 과열(OVP)
#define			ERR_PELT_OVER		2		//ER02 Peltier 과열(OVP)
#define			ERR_FAN_PTC			3		//ER03 FAN_Vial(PTC) Part Error
#define			ERR_FAN_PELT		4		//ER04 FAN_Peltier Part Error
#define			ERR_PTC_OPEN		5		//ER05 PTC Heater 단선
#define			ERR_PELT_OPEN		6		//ER06 Peltier 단선
#define			ERR_PTC_SENSOR		7		//ER07 PTC NTC 센서 단선/단락 (추가)
#define			ERR_PELT_SENSOR		8		//ER08 Peltier NTC 센서 단선/단락 (추가)

//================================================================
// 설정 범위 (HMI 입력값 : 온도 ℃ 정수, 시간 분)
//================================================================
#define			PTC_SET_MIN			25		//℃
#define			PTC_SET_MAX			120		//℃
#define			PELT_SET_MIN		5		//℃ (결로/결빙 방지)
#define			PELT_SET_MAX		70		//℃ (펠티어 손상 방지)

#define			TIME_SET_MIN		1		//분
#define			TIME_SET_MAX		10		//분  (0 = 사용안함 단계 -> 건너뜀)

//================================================================
// NTC 50K (B=3950)  /  분압 : DC5V - NTC - ADC - R(10K 3296 가변) - GND
//================================================================
#define			NTC_R25				50000.0	//25℃ 저항
#define			NTC_BETA			3950.0	//B 정수 (센서 데이터시트 값으로 수정)
#define			NTC_R_PULLDOWN		10000.0	//R3/R4/R5 3296 가변저항 실측값 입력

#define			NTC_ADC_OPEN		10		//이하 = 센서 단선 (약 -30℃)
#define			NTC_ADC_SHORT		1010	//이상 = 센서 단락
#define			TEMP_ERR			(-9999)	//센서 이상 표시값

//ADC 채널 순서 (AREF = DC5V 외부 기준)
#define			ADC_CH_NUM			5
#define			ADC_IDX_PTC			0		//ADC0 PF0 PTC_S1
#define			ADC_IDX_PELT		1		//ADC1 PF1 PELT_S2
#define			ADC_IDX_SPARE		2		//ADC2 PF2 SPARE_S3
#define			ADC_IDX_FAN_PTC		3		//ADC4 PF4 FAN_ADC1
#define			ADC_IDX_FAN_PELT	4		//ADC5 PF5 FAN_ADC2

//================================================================
// 공통 제어 (온도 단위 0.1℃)
//================================================================
#define			HOLD_IN_BAND		10		//1.0℃ 이내 도달하면 유지시간 카운트 시작
#define			RAMP_TIMEOUT_SEC	600		//10분 내 미도달시 유지시간 카운트 강제 시작
#define			PID_I_BAND			50		//5.0℃ 이내에서만 적분

//단선 감지 : 출력 90% 이상에서 일정시간 1.0℃ 변화 없으면 단선
#define			OPEN_CHK_OUT		900		//‰
#define			OPEN_CHK_DELTA		10		//1.0℃
#define			PTC_OPEN_TIME		60		//sec
#define			PELT_OPEN_TIME		120		//sec

//================================================================
// PTC 히터 (24V 50W 카트리지) : 소프트 PWM 10ms x 100 = 1sec 주기
//================================================================
#define			PTC_PWM_STEP		100		//1 step = 10ms
#define			PTC_DUTY_MAX		100		//% 최대 출력
#define			PTC_USE_OUT2		1		//1 = HT_OUT2 도 HT_OUT1 과 동일 출력

#define			PTC_KP				250		//out(‰) = KP*e/10    (4.0℃ 오차 -> 100%)
#define			PTC_KI				15		//      + KI*sum(e)/100
#define			PTC_KD				40		//      - KD*dPV(0.1℃/sec)

#define			PTC_OVER_TEMP		1350	//135.0℃ 이상 -> ER01
#define			PTC_FAN_ON_BAND		30		//PV > SV+3.0℃ -> Vial FAN ON (냉각)
#define			PTC_FAN_OFF_BAND	10		//PV <= SV+1.0℃ -> Vial FAN OFF
#define			PTC_FAN_ALWAYS		0		//1 = PTC 동작중 FAN 항상 ON

//================================================================
// Peltier (DC12V, H-Bridge, PWM 1kHz)  *** 부품 보호 초기값 ***
//================================================================
#define			PELT_PWM_TOP		1999	//16MHz/8/(1+1999) = 1kHz (PC817 응답속도 고려)
#define			PELT_COOL_DUTY_MAX	700		//‰ 냉각 최대 70%
#define			PELT_HEAT_DUTY_MAX	500		//‰ 가열 최대 50%
#define			PELT_DUTY_STEP_UP	10		//‰/100ms 소프트 스타트 (0->70% 약 7초)
#define			PELT_DUTY_STEP_DN	20		//‰/100ms 소프트 정지
#define			PELT_DEAD_TIME		50		//x100ms = 5초 : 극성 전환 전 OFF 유지 (열충격 방지)
#define			PELT_OUT_DEADBAND	30		//‰ 이하 출력은 OFF (잦은 극성전환 방지)
#define			PELT_FAN_PRE_SEC	3		//방열 FAN 먼저 3초 동작 후 펠티어 출력
#define			PELT_FAN_AFTER_SEC	60		//정지 후 방열 FAN 60초 추가 동작

#define			PELT_KP				200		//5.0℃ 오차 -> 100%
#define			PELT_KI				10
#define			PELT_KD				30

#define			PELT_OVER_TEMP		800		//80.0℃ 이상 -> ER02
#define			PELT_HS_SENSOR_USE	0		//1 = SPARE_S3 센서를 방열판에 부착하여 감시
#define			PELT_HS_OVER_TEMP	600		//방열판 60.0℃ 이상 -> ER02

#define			PELT_DIR_NONE		0
#define			PELT_DIR_HEAT		1		//Q9A + Q12 ON
#define			PELT_DIR_COOL		2		//Q10A + Q11 ON

//================================================================
// FAN (24V, 전류검출 4.7Ω x 48배 -> FAN 연결/단선 확인)
//================================================================
#define			FAN_CHECK_DELAY		3		//FAN ON 후 3초 후부터 검사
#define			FAN_OPEN_ADC		100		//이하 = FAN 전류 없음 (약 2mA)
#define			FAN_ERR_TIME		3		//3초 연속 -> 에러

//================================================================
// 포트
//================================================================
//PTC 히터 : PA6=PTC_OUT1, PA7=PTC_OUT2 (Q1 NPN 반전 -> LOW = 히터 ON)
#define			PTC1_ON				PORTA &= (~0x40)
#define			PTC1_OFF			PORTA |=   0x40
#define			PTC2_ON				PORTA &= (~0x80)
#define			PTC2_OFF			PORTA |=   0x80

//FAN : PA3=FAN_OUT1(FAN_PTC), PA4=FAN_OUT2(FAN_PELT) (Q5/Q6 PNP -> LOW = FAN ON)
#define			FAN_PTC_ON			PORTA &= (~0x08)
#define			FAN_PTC_OFF			PORTA |=   0x08
#define			FAN_PELT_ON			PORTA &= (~0x10)
#define			FAN_PELT_OFF		PORTA |=   0x10

//Peltier : PC4=HOT_ON/OFF(Q9A), PC5=COOL_ON/OFF(Q10A) HIGH = ON
//          PB5=HOT_PWM(OC1A, Q12), PE3=COOL_PWM(OC3A, Q11)
#define			HOT_ON				PORTC |=   0x10
#define			HOT_OFF				PORTC &= (~0x10)
#define			COOL_ON				PORTC |=   0x20
#define			COOL_OFF			PORTC &= (~0x20)

//LED D5 : PB0 (LOW = ON)
#define			LED_POWER_ON		PORTB &= (~0x01)
#define			LED_POWER_OFF		PORTB |=   0x01

//================================================================
// 통신 (USART1 = TOUCH_COM, USART0 = DEBUG_COM) 19200bps
//================================================================
#define			UBRR_19200			103		//U2X=1, 16MHz
#define			DEBUG_TX_USE		1		//1 = DEBUG_COM 으로도 동일 데이터 출력
#define			RX_BUF_MAX			50
#define			ARG_MAX				6

#define			TX_DAT				1
#define			TX_FAIL				2
#define			TX_OK				3
#define			TX_PT				4
#define			TX_END				5
#define			TX_ERR				6

//================================================================
// EEPROM
//================================================================
#define			INIT_DATA			0x5a31
#define			ADDR_INIT_H			0
#define			ADDR_INIT_L			1
#define			ADDR_PT				16		//[mode 2][PT 5][STEP 6] x 3byte(temp H,L,time)


void EEPROM_WriteByte(uint16_t Address, uint8_t Data);
uint8_t EEPROM_ReadByte(uint16_t Address);
void fcReadData(void);
void fcDataErr_Check(void);
void fcDataSave(uint8_t mode, uint8_t pt, uint8_t step);

void fcAdcConversion(void);
int16_t fcNtcTemp(uint16_t adc);
void fcProtectCheck(void);
void fcFanControl(void);
void fcFanCheck(void);

void fcRun(void);
void fcRunStart(uint8_t pt);
void fcRunStop(void);
void fcRunEnd(void);
void fcStepStart(uint8_t step);
void fcOpenCheck(int16_t pv);
void fcSetError(uint8_t err);
void fcAllOff(void);
int16_t fcPid(int16_t sv, int16_t pv, int16_t kp, int16_t ki, int16_t kd, int16_t outMin, int16_t outMax);

void fcPeltOutput(void);
void Pelt_Off(void);
void Pelt_Enable(uint8_t dir);
void Pelt_SetDuty(uint8_t dir, uint16_t duty);

void fcLedDisplay(void);

void Receive(void);
void Transfer(uint8_t type);
void UART_SendString1(uint8_t *str, uint8_t len);

#endif /* USER_DEFINE_H_ */
