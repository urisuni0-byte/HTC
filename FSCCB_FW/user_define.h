
/*
 * user_define.h : FSCCB_V1 (RED / UV LED 제어) - ATtiny24A-MU 내부 RC 8MHz
 */

#ifndef USER_DEFINE_H_
#define USER_DEFINE_H_

//================================================================
// 모드
//================================================================
#define			MODE_POWERON		0		//배터리 연결 후 1초 대기
#define			MODE_SLEEP			1		//Sleep (Power Down)
#define			MODE_WAKE			2		//Wake-up 후 터치/충전 확인
#define			MODE_RUN			3		//LED 동작 (1 -> 2 -> 3단계)
#define			MODE_LOWBAT			4		//배터리 부족 : 적색 LED 3회 점멸
#define			MODE_CHARGE			5		//충전중 : LED 10%
#define			MODE_END			6		//동작 종료, 터치 분리 대기

//================================================================
// 시간 (10ms 단위)
//================================================================
#define			T1SEC				100
#define			POWERON_TIME		T1SEC	//전원 공급 1초 후 Sleep
#define			WAKE_TIME			50		//Wake-up 후 0.5초 안에 터치/충전 없으면 다시 Sleep

#define			KEY_CHATTERING		5		//터치 감지 50ms
#define			TOUCH_OFF_TIME		50		//터치 분리 0.5초 (얼굴에서 잠깐 떨어지는 것 무시)
#define			CHARGE_CHATTERING	10		//충전 신호 100ms

#define			LED_RISE_TIME		100		//1초  Rising
#define			LED_ON_TIME			17800	//2분 58초 ON
#define			LED_FALL_TIME		100		//1초  Falling

#define			STEP_ON_TIME		(LED_RISE_TIME+LED_ON_TIME)	//Rising + ON
#define			STEP_OFF_TIME		LED_FALL_TIME				//Falling (교차 제어때는 다음 LED Rising 과 겹침)

#define			LOWBAT_BLINK_TIME	T1SEC	//1초 간격 ON/OFF
#define			LOWBAT_BLINK_NUM	3		//3번 반복

//================================================================
// PWM (Timer1, N=8 -> 1count = 1us)
//================================================================
#define			PWM_FREQ			300		//Hz (100 단위로 설정 : 200, 300 ...)
#define			PWM_TOP				((F_CPU/8/PWM_FREQ)-1)		//300Hz = 3332
#define			PWM_10MS_CNT		(PWM_FREQ/100)				//10ms = PWM 3주기

#define			DUTY_RUN			((PWM_TOP+1UL)*50/100)		//동작   50%
#define			DUTY_CHARGE			((PWM_TOP+1UL)*10/100)		//충전중 10%
#define			DUTY_MIN			30							//이하 출력 안함 (30us)

#define			DUTY_STEP_UP		((DUTY_RUN+LED_RISE_TIME-1)/LED_RISE_TIME)	//10ms 당 증가량
#define			DUTY_STEP_DN		((DUTY_RUN+LED_FALL_TIME-1)/LED_FALL_TIME)	//10ms 당 감소량

//================================================================
// LED 동작 순서
//================================================================
#define			LED_NONE			0
#define			LED_RED				1
#define			LED_UV				2

#define			SEQ_MAX				9

#define			CHARGE_LED_RED		1		//충전중 표시 LED : 1=RED , 0=UV

//================================================================
// 배터리 전압 : AVCC 기준으로 내부 1.1V 측정 -> ADC = 1.1V x 1024 / VBAT
//               (전압이 낮을수록 ADC 값이 커짐)
//================================================================
#define			BG_MV				1100UL	//내부 기준전압 mV (편차 있으면 실측값으로 보정)
#define			BAT_LOW_MV			3000UL	//이하 -> 동작 정지
#define			BAT_OK_MV			3300UL	//이상 -> 배터리 부족 해제

#define			BAT_LOW_ADC			(BG_MV*1024UL/BAT_LOW_MV)	//375
#define			BAT_OK_ADC			(BG_MV*1024UL/BAT_OK_MV)	//341

#define			BAT_AVR_CNT			8		//10ms x 8 평균
#define			BAT_SKIP_CNT		2		//ADC ON 후 처음 2회 버림
#define			BAT_LOW_CNT			5		//평균값 5회(0.4초) 연속 -> 배터리 부족

//================================================================
// 포트
//================================================================
//PA7 = RED_PWM (U4 MIC2843 END) , PB2 = UVL_PWM (U2 MIC2843 END) : HIGH = LED ON
#define			RED_ON				PORTA |=   0x80
#define			RED_OFF				PORTA &= (~0x80)
#define			UV_ON				PORTB |=   0x04
#define			UV_OFF				PORTB &= (~0x04)

//PA3 = ON/OFF_CNT (AWS01M OUT, R4 풀업) : LOW = 터치
#define			TOUCH_IN			(!(PINA & 0x08))

//PA1 = CH_/CNT (MCP73831 STAT, R16 풀업) : LOW = 충전중 , HIGH = 충전완료/충전기 없음
#define			CHARGE_IN			(!(PINA & 0x02))


void cpu_init(void);
void feed_dog(void);
void Timer1_Init(void);
void Pwm_Run(void);
void Pwm_Stop(void);
void Adc_Init(void);

void fcKeyScan(void);
void fcBatCheck(void);
void fcRun(void);
void fcRunStart(void);
void fcSeqSet(void);
void fcSleep(void);

void Led_Set(uint16_t red, uint16_t uv);
void fcLedRamp(void);
uint16_t fcRamp(uint16_t now, uint16_t set);

#endif /* USER_DEFINE_H_ */
