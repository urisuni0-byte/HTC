/*
 * FSCCB.c : RED / UV LED 제어
 * MCU : ATtiny24A-MU 내부 RC 8MHz  (FSCCB_V1)
 */

#define F_CPU 8000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include "user_define.h"

// LED 동작 순서 ------------------------------------------------------
//                            1단계               2단계              3단계 (RED <-> UV 교차 제어)
const uint8_t  SEQ_LED[SEQ_MAX]  = { LED_RED     , LED_NONE     , LED_UV      , LED_NONE     , LED_RED     , LED_UV      , LED_RED     , LED_UV      , LED_NONE      };
const uint16_t SEQ_TIME[SEQ_MAX] = { STEP_ON_TIME, STEP_OFF_TIME, STEP_ON_TIME, STEP_OFF_TIME, STEP_ON_TIME, STEP_ON_TIME, STEP_ON_TIME, STEP_ON_TIME, STEP_OFF_TIME };

//timer
volatile uint8_t bPwmCnt=0,b10msFg=0;

//pwm duty (ISR 에서 사용)
volatile uint16_t wRedDuty=0;
volatile uint16_t wUvDuty=0;

//led
uint16_t wRedSet=0;		//목표 duty
uint16_t wUvSet=0;

//key
uint8_t bTouchFg=0,bTouchOnCnt=0,bTouchOffCnt=0;
uint8_t bChargeFg=0,bChargeCnt=0;

//mode
uint8_t  wSysMode=MODE_POWERON;
uint8_t  bSeq=0;
uint16_t wStepCnt=0;
uint8_t  bBlinkCnt=0;

//battery
uint16_t wAdcSum=0;
uint8_t  bAdcCnt=0,bAdcSkip=0;
uint8_t  bLowCnt=0;
uint8_t  bLowBatFg=0;
uint8_t  bBatOkFg=0;	//Wake-up 후 전압 측정 완료


ISR(TIM1_CAPT_vect)   //PWM 주기 시작 (300Hz = 3.33ms)
{
	OCR1A = wRedDuty;
	OCR1B = wUvDuty;

	if(wRedDuty >= DUTY_MIN)	RED_ON;
	if(wUvDuty  >= DUTY_MIN)	UV_ON;

	if(++bPwmCnt >= PWM_10MS_CNT)//10ms timer
	{
		bPwmCnt = 0 ;
		b10msFg = 1 ;
	}
}

ISR(TIM1_COMPA_vect)  //RED duty 끝
{
	RED_OFF;
}

ISR(TIM1_COMPB_vect)  //UV duty 끝
{
	UV_OFF;
}

ISR(PCINT0_vect)      //터치 / 충전 신호 변화 -> Wake-up
{

}


void Timer1_Init(void)
{
 // 16bit Timer1/Counter1
 // CTC Mode  Mode 12  WGM13:WGM10  - 1100  (TOP = ICR1)
 // F= fclk / N*(1+TOP) => 8000000 / 8*(1+3332) = 300Hz
 // PA7 , PB2 는 OC1A/OC1B 핀이 아니므로 인터럽트에서 포트 출력
	TCCR1A = 0x00;
	TCCR1B = 0x00;

	ICR1   = PWM_TOP;
	OCR1A  = 0;
	OCR1B  = 0;
	TCNT1  = 0x0000;

	TIMSK1 = (1<<ICIE1)|(1<<OCIE1B)|(1<<OCIE1A);
}

void Pwm_Run(void)
{
	TCNT1  = 0 ;
	TCCR1B = (1<<WGM13) |(1<<WGM12 )|(0<<CS12  )|(1<<CS11 )|(0<<CS10 );    // N=8
}

void Pwm_Stop(void)
{
	TCCR1B = 0x00;

	wRedDuty = 0;	wRedSet = 0;
	wUvDuty  = 0;	wUvSet  = 0;

	RED_OFF;
	UV_OFF;
}

void Adc_Init(void)
{
	ADMUX  = 0x21;   // 기준 = VCC , 입력 = 내부 1.1V
	ADCSRA = (1<<ADEN)|(1<<ADSC)|(1<<ADPS2)|(1<<ADPS1); // 8MHz/64 = 125kHz

	wAdcSum  = 0;
	bAdcCnt  = 0;
	bAdcSkip = BAT_SKIP_CNT;
	bLowCnt  = 0;
	bBatOkFg = 0;
}

void cpu_init(void)
{
	MCUSR = 0;
	wdt_disable();

	CLKPR = 0x80;  // 시스템 클럭 8MHz (CKDIV8 퓨즈와 무관)
	CLKPR = 0x00;

	//port init //1=OUT , 0=IN
	//PA7=RED_PWM(OUT) PA6,PA5,PA4=ISP(풀업) PA3=ON/OFF_CNT PA2=NC(풀업) PA1=CH_/CNT PA0=ADC0
	PORTA = 0x74 ; // 01110100
	DDRA  = 0x80 ; // 10000000

	//PB2=UVL_PWM(OUT) PB1=NC(풀업) PB0=GND_CNT(Hi-Z : 분압저항 전류 차단)
	PORTB = 0x02 ; // 00000010
	DDRB  = 0x04 ; // 00000100

	DIDR0 = 0x01 ; // PA0 디지털 입력 차단
	PRR   = (1<<PRTIM0)|(1<<PRUSI);

	//pin change : PA3(PCINT3) 터치 , PA1(PCINT1) 충전
	PCMSK0 = (1<<PCINT3)|(1<<PCINT1);
	GIMSK  = (1<<PCIE0);

	Timer1_Init();
	Adc_Init();

	Pwm_Stop();
	Pwm_Run();

	wdt_enable(WDTO_500MS); // watchdog 0.5sec
}

void feed_dog(void)
{
	wdt_reset();
}


int main(void)
{
	cpu_init();

	sei();

	while(1)
	{
		feed_dog() ;

		if( b10msFg )
		{
			b10msFg=0;

			fcKeyScan();
			fcBatCheck();
			fcRun();
			fcLedRamp();
		}

		if(wSysMode == MODE_SLEEP)
		{
			fcSleep();
		}
	}
}

void fcSleep(void)
{
	Pwm_Stop();
	ADCSRA = 0x00;  // ADC off
	wdt_disable();

	cli();
	if(!TOUCH_IN && !CHARGE_IN)
	{
		set_sleep_mode(SLEEP_MODE_PWR_DOWN);
		sleep_enable();
		sleep_bod_disable();
		sei();
		sleep_cpu();   // 터치 / 충전 신호 변화까지 정지
		sleep_disable();
	}
	sei();

	//wake up
	wdt_enable(WDTO_500MS);

	bTouchFg  = 0;	bTouchOnCnt = 0;	bTouchOffCnt = 0;
	bChargeFg = 0;	bChargeCnt  = 0;

	Adc_Init();
	Pwm_Run();

	wStepCnt = 0;
	wSysMode = MODE_WAKE;
}

void fcRunStart(void)
{
	bSeq = 0;
	wStepCnt = 0;
	fcSeqSet();

	wSysMode = MODE_RUN;
}

void fcSeqSet(void)
{
	wRedSet = 0;
	wUvSet  = 0;

	if(SEQ_LED[bSeq] == LED_RED)		wRedSet = DUTY_RUN;
	else if(SEQ_LED[bSeq] == LED_UV)	wUvSet  = DUTY_RUN;
}

void fcRun(void)//10ms
{
	//충전기 연결 -> 동작 정지 , LED 10%
	if(bChargeFg && (wSysMode != MODE_POWERON) && (wSysMode != MODE_CHARGE))
	{
		if(CHARGE_LED_RED)	Led_Set(DUTY_CHARGE, 0);
		else				Led_Set(0, DUTY_CHARGE);

		wSysMode = MODE_CHARGE;
	}

	switch(wSysMode) {
		case MODE_POWERON:
			if(++wStepCnt >= POWERON_TIME) {// 1초 후 Sleep
				wSysMode = MODE_SLEEP;
			}
		break;
		case MODE_WAKE:
			if(bTouchFg && bBatOkFg)
			{
				if(bLowBatFg){
					wStepCnt = 0;
					bBlinkCnt = 0;
					wSysMode = MODE_LOWBAT;
				}
				else{
					fcRunStart();
				}
			}
			else if(++wStepCnt >= WAKE_TIME) {
				wSysMode = MODE_SLEEP;
			}
		break;
		case MODE_RUN:
			if(!bTouchFg) {//터치 분리 -> Sleep
				wSysMode = MODE_SLEEP;
			}
			else if(bLowBatFg) {//배터리 3V 이하 -> 동작 정지
				wStepCnt = 0;
				bBlinkCnt = 0;
				wSysMode = MODE_LOWBAT;
			}
			else if(++wStepCnt >= SEQ_TIME[bSeq]) {
				wStepCnt = 0;

				if(++bSeq >= SEQ_MAX) {//3단계 끝
					Led_Set(0, 0);
					wSysMode = MODE_END;
				}
				else{
					fcSeqSet();
				}
			}
		break;
		case MODE_LOWBAT: //적색 LED 1초 간격 ON/OFF 3번 후 OFF
			if(bBlinkCnt & 0x01)	Led_Set(0, 0);
			else					Led_Set(DUTY_RUN, 0);

			if(++wStepCnt >= LOWBAT_BLINK_TIME) {
				wStepCnt = 0;

				if(++bBlinkCnt >= (LOWBAT_BLINK_NUM*2)) {
					Led_Set(0, 0);
					wSysMode = MODE_END;
				}
			}
		break;
		case MODE_CHARGE:
			if(!bChargeFg) {//충전 완료 (또는 충전기 분리) -> LED OFF
				Led_Set(0, 0);
				wSysMode = MODE_END;
			}
		break;
		case MODE_END: //터치 분리되면 Sleep
			if(!bTouchFg) {
				wSysMode = MODE_SLEEP;
			}
		break;
		default :
			wSysMode = MODE_SLEEP;
		break;
	}
}


void fcKeyScan(void)//10ms
{
	//touch
	if(TOUCH_IN)
	{
		bTouchOffCnt = 0;

		if(!bTouchFg && (++bTouchOnCnt >= KEY_CHATTERING))
		{
			bTouchFg = 1;
		}
	}
	else
	{
		bTouchOnCnt = 0;

		if(bTouchFg && (++bTouchOffCnt >= TOUCH_OFF_TIME))
		{
			bTouchFg = 0;
		}
	}

	//charge
	if(CHARGE_IN)
	{
		if(bChargeFg)	bChargeCnt = 0;
		else if(++bChargeCnt >= CHARGE_CHATTERING)
		{
			bChargeCnt = 0;
			bChargeFg = 1;
		}
	}
	else
	{
		if(!bChargeFg)	bChargeCnt = 0;
		else if(++bChargeCnt >= CHARGE_CHATTERING)
		{
			bChargeCnt = 0;
			bChargeFg = 0;
		}
	}
}

void fcBatCheck(void)//10ms
{
	uint16_t adc;

	adc = ADC;
	ADCSRA |= (1<<ADSC);

	if(bAdcSkip)
	{
		bAdcSkip--;
		return;
	}

	wAdcSum += adc;

	if(++bAdcCnt >= BAT_AVR_CNT)
	{
		adc = wAdcSum / BAT_AVR_CNT;
		wAdcSum = 0;
		bAdcCnt = 0;

		if(adc >= BAT_LOW_ADC) // 3.0V 이하
		{
			if(++bLowCnt >= BAT_LOW_CNT)
			{
				bLowCnt = BAT_LOW_CNT;
				bLowBatFg = 1;
			}
		}
		else
		{
			bLowCnt = 0;

			if(adc <= BAT_OK_ADC)	bLowBatFg = 0; // 3.3V 이상
		}

		bBatOkFg = 1;
	}
}


//즉시 출력 (Rising / Falling 없음)
void Led_Set(uint16_t red, uint16_t uv)
{
	wRedSet = red;
	wUvSet  = uv;

	cli();
	wRedDuty = red;
	wUvDuty  = uv;
	sei();
}

uint16_t fcRamp(uint16_t now, uint16_t set)
{
	if(now < set)
	{
		now += DUTY_STEP_UP;
		if(now > set)	now = set;
	}
	else if(now > set)
	{
		if(now > (set + DUTY_STEP_DN))	now -= DUTY_STEP_DN;
		else							now = set;
	}

	return now;
}

//1초 Rising / 1초 Falling
void fcLedRamp(void)//10ms
{
	uint16_t red, uv;

	red = fcRamp(wRedDuty, wRedSet);
	uv  = fcRamp(wUvDuty , wUvSet );

	cli();
	wRedDuty = red;
	wUvDuty  = uv;
	sei();
}
