
/*
 * HTC main.c : PTC / Peltier 6단계 온도 프로토콜 제어
 * MCU : ATmega128A-AU 16MHz  (VSHCCB_V1)
 */
//261030
//
// Fuse : M103C 해제(필수), JTAGEN 해제(필수 : PF4/PF5 FAN_ADC 사용), BODEN 4.0V, 외부 16MHz 크리스탈
// Linker : libm 포함 (log 사용)

#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <math.h>
#include "user_define.h"


//timer
volatile uint8_t b10msCnt=0,b10msFg=0;
volatile uint8_t b100msCnt=0,b100msFg=0;
volatile uint8_t b1sCnt=0,b1sFg=0;
uint8_t b500msCnt = 0;
uint8_t bLedFg = 0;

//adc
const uint8_t bAdcChTbl[ADC_CH_NUM] = { 0x00, 0x01, 0x02, 0x04, 0x05 }; //REFS=00 (AREF=DC5V)
volatile uint8_t  bAdcFg=0, bAdcCnt=0, bAdcCh=0;
volatile uint16_t wAdcAcc[ADC_CH_NUM]={0,};
volatile uint16_t wADC_Sum[ADC_CH_NUM]={0,};
uint16_t wAdc[ADC_CH_NUM]={0,};

//temp (0.1℃)
int16_t iTempPtc  = TEMP_ERR;
int16_t iTempPelt = TEMP_ERR;
int16_t iTempHs   = TEMP_ERR;
uint8_t bSensorErrCnt=0;

//protocol [mode][pt][step]
int16_t iPtTemp[2][PT_MAX][STEP_MAX];
uint8_t bPtTime[2][PT_MAX][STEP_MAX];

const int16_t iDefTemp[2][STEP_MAX] = {
	{ 50, 60, 70, 80, 60, 40 },		//PTC 기본값 ℃
	{ 25, 15, 10, 20, 37, 25 }		//Peltier 기본값 ℃
};
const uint8_t bDefTime[2][STEP_MAX] = {
	{ 2, 2, 3, 3, 2, 1 },			//PTC 기본값 분
	{ 2, 3, 3, 3, 3, 2 }			//Peltier 기본값 분
};

//run
uint8_t  bMode=MODE_NONE;
uint8_t  bRunState=ST_IDLE;
uint8_t  bPt=0, bStep=0;
int16_t  iSetTemp=0;			//0.1℃
uint16_t wRemainSec=0;
uint16_t wRampSec=0;
uint8_t  bHoldFg=0;
uint32_t lRunSec=0;
int16_t  iOut=0;				//PID 출력 ‰ (PTC 0~1000, Peltier -1000~1000)
uint8_t  bError=ERR_NONE;

//pid
int32_t  lPidSum=0;
int16_t  iPidPvOld=TEMP_ERR;

//단선 감지
uint8_t  bOpenChkDone=0;
uint16_t wOpenCnt=0;
int16_t  iOpenPv=0;

//ptc
volatile uint8_t bPtcDuty=0;	//0~100 (ISR 소프트 PWM)
volatile uint8_t bPtcPwmCnt=0;

//peltier
uint8_t  bPeltDir=PELT_DIR_NONE;
uint16_t wPeltDuty=0;			//현재 duty ‰
int16_t  iPeltTarget=0;			//PID 요구 ‰
uint8_t  bPeltDeadCnt=0;

//fan
uint8_t  bFanPtcFg=0, bFanPeltFg=0;
uint8_t  bFanPtcReq=0;
uint8_t  bFanPtcOnSec=0, bFanPeltOnSec=0;
uint8_t  bFanPtcErrCnt=0, bFanPeltErrCnt=0;
uint16_t wPeltFanAfterSec=0;

//comm
uint8_t len=0;
uint8_t bRxData[RX_BUF_MAX]={0,};
uint8_t bRx[RX_BUF_MAX]={0,};
char    bCmd[RX_BUF_MAX+1];
uint8_t TxData[80];
volatile uint8_t bRxCnt=0,bRxLen=0,bRxOk=0;
uint8_t rxbuf=0,bRxSeq=0;
int16_t iArg[ARG_MAX];
uint8_t bGetMode=0, bGetPt=0;


ISR(TIMER2_COMP_vect)   //1ms
{
	if(++b10msCnt >= 10)//10ms timer
	{
		b10msCnt = 0 ;
		b10msFg = 1 ;

		//PTC 소프트 PWM 10ms x 100 = 1sec
		if(++bPtcPwmCnt >= PTC_PWM_STEP)	bPtcPwmCnt = 0;

		if(bPtcPwmCnt < bPtcDuty){
			PTC1_ON;
#if PTC_USE_OUT2
			PTC2_ON;
#endif
		}
		else{
			PTC1_OFF;
			PTC2_OFF;
		}

		//ADC 10ms
		bAdcCh = 0;
		ADMUX = bAdcChTbl[0];
		ADCSRA |= 1<<ADSC; //A/D START

		if(++b100msCnt >=10){
			b100msCnt=0;
			b100msFg=1;

			if(++b1sCnt >= 10){
				b1sCnt=0;
				b1sFg=1;
			}
		}
	}
}


//timer 1ms
void Timer2_Init(void)
{
	TCCR2 = (1<<WGM21)|(0<<CS22)|(1<<CS21)|(1<<CS20); // CTC 64분주
	OCR2 = 249 ;// 16MHz/64/(1+249) = 1000Hz -> 1ms
	TCNT2 = 0;
	TIMSK |= (1<<OCIE2);
}

//HOT_PWM : OC1A(PB5) Fast PWM Mode 14, TOP=ICR1
void Timer1_Init(void)
{
	TCCR1A = (0<<COM1A1)|(0<<COM1A0)|(1<<WGM11)|(0<<WGM10); //출력 분리 (duty 0 일때 완전 OFF)
	TCCR1B = (1<<WGM13)|(1<<WGM12)|(0<<CS12)|(1<<CS11)|(0<<CS10); // N=8, 16MHz/8/(1+1999) = 1kHz

	ICR1  = PELT_PWM_TOP;
	OCR1A = 0;
	TCNT1 = 0;
}

//COOL_PWM : OC3A(PE3) Fast PWM Mode 14, TOP=ICR3
void Timer3_Init(void)
{
	TCCR3A = (0<<COM3A1)|(0<<COM3A0)|(1<<WGM31)|(0<<WGM30);
	TCCR3B = (1<<WGM33)|(1<<WGM32)|(0<<CS32)|(1<<CS31)|(0<<CS30); // N=8, 1kHz

	ICR3  = PELT_PWM_TOP;
	OCR3A = 0;
	TCNT3 = 0;
}

//USART1 : TOUCH_COM(HMI) , USART0 : DEBUG_COM
void UART_init(void)
{
	UCSR1A = (1 << U2X1); // x2
	UCSR1B = (1 << RXCIE1) | (1 << RXEN1) | (1 << TXEN1);
	UCSR1C = (1 << UCSZ11) | (1 << UCSZ10); // 8bit, 1stop, no parity
	UBRR1H = 0;
	UBRR1L = UBRR_19200;

	UCSR0A = (1 << U2X0);
	UCSR0B = (1 << TXEN0);
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
	UBRR0H = 0;
	UBRR0L = UBRR_19200;
}


void ADC_Init()
{
	ADCSRA = (1<<ADEN) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0) | (1<<ADIE); // 16MHz/128 = 125kHz
	ADMUX  = bAdcChTbl[0];
}

ISR(ADC_vect)
{
	uint8_t i;

	wAdcAcc[bAdcCh] += (ADC & 0x03ff);

	if(++bAdcCh < ADC_CH_NUM)
	{
		ADMUX = bAdcChTbl[bAdcCh];
		ADCSRA |= 1<<ADSC; //A/D START
	}
	else
	{
		if(++bAdcCnt >= 10) //100ms마다
		{
			bAdcCnt = 0;

			for(i=0;i<ADC_CH_NUM;i++){
				wADC_Sum[i] = wAdcAcc[i];
				wAdcAcc[i]  = 0;
			}
			bAdcFg = 1;
		}
	}
}


//================================================================
// Peltier H-Bridge
//  HEAT : HOT_ON(Q9A) + HOT_PWM(Q12)
//  COOL : COOL_ON(Q10A) + COOL_PWM(Q11)
//  HOT_ON + COOL_PWM 또는 COOL_ON + HOT_PWM 동시 ON = 단락 -> 절대 금지
//================================================================
void Pelt_Off(void)
{
	TCCR1A &= ~(1<<COM1A1);	//PWM 출력 분리
	OCR1A = 0;
	PORTB &= (~0x20);		//HOT_PWM LOW

	TCCR3A &= ~(1<<COM3A1);
	OCR3A = 0;
	PORTE &= (~0x08);		//COOL_PWM LOW

	HOT_OFF;
	COOL_OFF;
}

void Pelt_Enable(uint8_t dir)
{
	Pelt_Off();			//반대쪽 완전 OFF 후

	if(dir == PELT_DIR_HEAT)		HOT_ON;
	else if(dir == PELT_DIR_COOL)	COOL_ON;
}

void Pelt_SetDuty(uint8_t dir, uint16_t duty)
{
	uint16_t ocr;

	ocr = (uint32_t)duty * PELT_PWM_TOP / 1000;

	if(dir == PELT_DIR_HEAT)
	{
		//반대 레그 OFF 확인
		TCCR3A &= ~(1<<COM3A1);
		OCR3A = 0;
		PORTE &= (~0x08);
		COOL_OFF;

		if(ocr == 0){
			TCCR1A &= ~(1<<COM1A1);
			OCR1A = 0;
			PORTB &= (~0x20);
		}
		else{
			OCR1A = ocr;
			TCCR1A |= (1<<COM1A1);
		}
	}
	else if(dir == PELT_DIR_COOL)
	{
		TCCR1A &= ~(1<<COM1A1);
		OCR1A = 0;
		PORTB &= (~0x20);
		HOT_OFF;

		if(ocr == 0){
			TCCR3A &= ~(1<<COM3A1);
			OCR3A = 0;
			PORTE &= (~0x08);
		}
		else{
			OCR3A = ocr;
			TCCR3A |= (1<<COM3A1);
		}
	}
	else
	{
		Pelt_Off();
	}
}

//100ms : 소프트 스타트/정지 + 극성 전환 데드타임
void fcPeltOutput(void)
{
	uint8_t  bReqDir;
	uint16_t wReqDuty;
	int16_t  target;

	target = iPeltTarget;

	if(bMode != MODE_PELT || bRunState != ST_RUN || bError || bFanPeltOnSec < PELT_FAN_PRE_SEC){
		target = 0;
	}

	if(target >= PELT_OUT_DEADBAND){
		bReqDir  = PELT_DIR_HEAT;
		wReqDuty = (uint32_t)target * PELT_HEAT_DUTY_MAX / 1000;
	}
	else if(target <= -PELT_OUT_DEADBAND){
		bReqDir  = PELT_DIR_COOL;
		wReqDuty = (uint32_t)(-target) * PELT_COOL_DUTY_MAX / 1000;
	}
	else{
		bReqDir  = PELT_DIR_NONE;
		wReqDuty = 0;
	}

	//방향 전환 or 정지 : duty 0 까지 감소 -> OFF -> 데드타임
	if(bPeltDir != PELT_DIR_NONE && bReqDir != bPeltDir)
	{
		if(wPeltDuty > PELT_DUTY_STEP_DN)	wPeltDuty -= PELT_DUTY_STEP_DN;
		else								wPeltDuty = 0;

		Pelt_SetDuty(bPeltDir, wPeltDuty);

		if(wPeltDuty == 0){
			Pelt_Off();
			bPeltDir = PELT_DIR_NONE;
			bPeltDeadCnt = PELT_DEAD_TIME;
		}
		return;
	}

	if(bPeltDir == PELT_DIR_NONE)
	{
		if(bPeltDeadCnt){
			bPeltDeadCnt--;
			return;
		}
		if(bReqDir == PELT_DIR_NONE)	return;

		Pelt_Enable(bReqDir);
		bPeltDir  = bReqDir;
		wPeltDuty = 0;
	}

	//소프트 램프
	if(wPeltDuty < wReqDuty){
		wPeltDuty += PELT_DUTY_STEP_UP;
		if(wPeltDuty > wReqDuty)	wPeltDuty = wReqDuty;
	}
	else if(wPeltDuty > wReqDuty){
		if(wPeltDuty > wReqDuty + PELT_DUTY_STEP_DN)	wPeltDuty -= PELT_DUTY_STEP_DN;
		else											wPeltDuty = wReqDuty;
	}

	Pelt_SetDuty(bPeltDir, wPeltDuty);
}


//모든 출력 OFF (FAN 제외)
void fcAllOff(void)
{
	bPtcDuty = 0;
	PTC1_OFF;
	PTC2_OFF;

	Pelt_Off();
	bPeltDir = PELT_DIR_NONE;
	wPeltDuty = 0;
	iPeltTarget = 0;
	bPeltDeadCnt = PELT_DEAD_TIME;

	iOut = 0;
}


void fcLedDisplay(void)
{
	uint8_t blink;

	if(bError){
		LED_POWER_ON;	//에러 : 계속 ON
		return;
	}

	blink = (bRunState == ST_RUN) ? 1 : 5;	//동작 200ms, 대기 1sec 점멸

	if(++b500msCnt >= blink)
	{
		b500msCnt = 0;
		bLedFg = ~bLedFg;
	}

	if(bLedFg)	LED_POWER_ON;
	else		LED_POWER_OFF;
}


//NTC 50K : ADC -> 0.1℃
int16_t fcNtcTemp(uint16_t adc)
{
	float r, t;

	if(adc < NTC_ADC_OPEN || adc > NTC_ADC_SHORT)	return TEMP_ERR;

	r = NTC_R_PULLDOWN * (1024.0 - (float)adc) / (float)adc;
	t = 1.0 / ( (1.0/298.15) + (log(r/NTC_R25) / NTC_BETA) ) - 273.15;

	if(t >= 0)	return (int16_t)(t*10.0 + 0.5);
	else		return (int16_t)(t*10.0 - 0.5);
}

int16_t fcTempFilter(int16_t old, int16_t now)
{
	if(now == TEMP_ERR || old == TEMP_ERR)	return now;
	return (int16_t)(((int32_t)old + now) / 2);
}

void fcAdcConversion(void)
{
	uint8_t i;

	if(bAdcFg)//100ms
	{
		bAdcFg=0;

		cli();
		for(i=0;i<ADC_CH_NUM;i++)	wAdc[i] = wADC_Sum[i] / 10;
		sei();

		iTempPtc  = fcTempFilter(iTempPtc,  fcNtcTemp(wAdc[ADC_IDX_PTC]));
		iTempPelt = fcTempFilter(iTempPelt, fcNtcTemp(wAdc[ADC_IDX_PELT]));
		iTempHs   = fcTempFilter(iTempHs,   fcNtcTemp(wAdc[ADC_IDX_SPARE]));
	}
}


void fcSetError(uint8_t err)
{
	if(bError)	return;		//최초 에러 유지

	bError = err;
	fcAllOff();
	bRunState = ST_ERROR;

	Transfer(TX_ERR);
}

//100ms
void fcProtectCheck(void)
{
	uint8_t sensor_err = 0;

	//과열 : 항상 감시 (FET 단락 등)
	if(iTempPtc != TEMP_ERR && iTempPtc >= PTC_OVER_TEMP)		fcSetError(ERR_PTC_OVER);
	if(iTempPelt != TEMP_ERR && iTempPelt >= PELT_OVER_TEMP)	fcSetError(ERR_PELT_OVER);
#if PELT_HS_SENSOR_USE
	if(iTempHs != TEMP_ERR && iTempHs >= PELT_HS_OVER_TEMP)		fcSetError(ERR_PELT_OVER);
	if(bMode == MODE_PELT && bRunState == ST_RUN && iTempHs == TEMP_ERR)	sensor_err = ERR_PELT_SENSOR;
#endif

	//센서 : 동작중 해당 모드만
	if(bRunState == ST_RUN || bRunState == ST_PAUSE)
	{
		if(bMode == MODE_PTC  && iTempPtc  == TEMP_ERR)	sensor_err = ERR_PTC_SENSOR;
		if(bMode == MODE_PELT && iTempPelt == TEMP_ERR)	sensor_err = ERR_PELT_SENSOR;
	}

	if(sensor_err){
		if(++bSensorErrCnt >= 5)	fcSetError(sensor_err); //500ms
	}
	else bSensorErrCnt = 0;
}

//100ms
void fcFanControl(void)
{
	uint8_t on;

	//PTC (Vial) FAN ---------------------------
	on = 0;
	if(bError == ERR_FAN_PTC)			on = 0;
	else if(bError == ERR_PTC_OVER)		on = 1;		//과열시 냉각
	else if(bMode == MODE_PTC && bRunState == ST_RUN)
	{
		if(iTempPtc != TEMP_ERR){
			if(iTempPtc > iSetTemp + PTC_FAN_ON_BAND)			bFanPtcReq = 1;
			else if(iTempPtc <= iSetTemp + PTC_FAN_OFF_BAND)	bFanPtcReq = 0;
		}
		on = bFanPtcReq;
#if PTC_FAN_ALWAYS
		on = 1;
#endif
	}
	else bFanPtcReq = 0;

	if(on)	FAN_PTC_ON;
	else	FAN_PTC_OFF;
	bFanPtcFg = on;

	//Peltier 방열 FAN -------------------------
	on = 0;
	if(bError == ERR_FAN_PELT)		on = 0;
	else if(bMode == MODE_PELT && (bRunState == ST_RUN || bRunState == ST_PAUSE)){
		on = 1;
		wPeltFanAfterSec = PELT_FAN_AFTER_SEC;
	}
	else if(wPeltFanAfterSec)				on = 1;		//정지 후 방열
	else if(bError == ERR_PELT_OVER)		on = 1;
	else if(bPeltDir != PELT_DIR_NONE)		on = 1;		//안전

	if(on)	FAN_PELT_ON;
	else	FAN_PELT_OFF;
	bFanPeltFg = on;
}

//1sec : FAN 전류 확인 (4.7Ω x 48배 -> 정상 동작시 ADC 포화)
void fcFanCheck(void)
{
	if(bFanPtcFg){
		if(bFanPtcOnSec < 250)	bFanPtcOnSec++;

		if(bFanPtcOnSec >= FAN_CHECK_DELAY && wAdc[ADC_IDX_FAN_PTC] < FAN_OPEN_ADC){
			if(++bFanPtcErrCnt >= FAN_ERR_TIME)	fcSetError(ERR_FAN_PTC);
		}
		else bFanPtcErrCnt = 0;
	}
	else{
		bFanPtcOnSec = 0;
		bFanPtcErrCnt = 0;
	}

	if(bFanPeltFg){
		if(bFanPeltOnSec < 250)	bFanPeltOnSec++;

		if(bFanPeltOnSec >= FAN_CHECK_DELAY && wAdc[ADC_IDX_FAN_PELT] < FAN_OPEN_ADC){
			if(++bFanPeltErrCnt >= FAN_ERR_TIME)	fcSetError(ERR_FAN_PELT);
		}
		else bFanPeltErrCnt = 0;
	}
	else{
		bFanPeltOnSec = 0;
		bFanPeltErrCnt = 0;
	}

	if(!(bMode == MODE_PELT && (bRunState == ST_RUN || bRunState == ST_PAUSE))){
		if(wPeltFanAfterSec)	wPeltFanAfterSec--;
	}
}


void cpu_init(void)
{
	//port init //1=OUT , 0=IN
	//PTC 히터는 리셋(Hi-Z) 상태에서 ON 되는 회로 -> 가장 먼저 OFF(HIGH) 출력
	PORTA = 0xd8; // 11011000  PTC2,PTC1,FAN2,FAN1 = HIGH(OFF)
	DDRA  = 0xd8; // 11011000

	PORTB = 0x01; // 00000001  LED OFF, HOT_PWM LOW
	DDRB  = 0x21; // 00100001  PB5 HOT_PWM, PB0 LED

	PORTC = 0x00; // HOT_ON/OFF, COOL_ON/OFF LOW(OFF)
	DDRC  = 0x30; // 00110000

	PORTD = 0x00;
	DDRD  = 0x08; // PD3 TXD1

	PORTE = 0x00; // COOL_PWM LOW
	DDRE  = 0x0a; // 00001010  PE3 COOL_PWM, PE1 TXD0

	PORTF = 0x00; // ADC (풀업 없음)
	DDRF  = 0x00;

	LED_POWER_OFF;

	Timer1_Init();
	Timer3_Init();
	Timer2_Init();

	ADC_Init();
	UART_init();

	Pelt_Off();
	fcAllOff();
	bPeltDeadCnt = 0;

	wdt_enable(WDTO_500MS);
}

void feed_dog(void)
{
	wdt_reset();
}

int main(void)
{
	cpu_init();

	fcReadData();

	sei();

	while(1)
	{
		feed_dog() ;

		fcAdcConversion() ;

		if( b10msFg )
		{
			b10msFg=0;

			Receive();
		}

		if(b100msFg)
		{
			b100msFg = 0;

			fcProtectCheck();

			fcFanControl();

			fcPeltOutput();

			fcLedDisplay();
		}

		if(b1sFg)
		{
			b1sFg = 0;

			fcFanCheck();

			fcRun();

			Transfer(TX_DAT);	//그래프 데이터 1sec
		}
	}
}


//================================================================
// 프로토콜 실행
//================================================================
void fcRunStart(uint8_t pt)
{
	bPt = pt;
	lRunSec = 0;
	bFanPtcReq = 0;
	bRunState = ST_RUN;

	fcStepStart(0);
}

void fcRunStop(void)
{
	fcAllOff();
	if(bRunState != ST_ERROR)	bRunState = ST_IDLE;
}

void fcRunEnd(void)
{
	fcAllOff();
	bRunState = ST_END;
	wRemainSec = 0;

	Transfer(TX_END);	//HMI : 부저 3회 + 데이터 저장
}

//다음 유효 단계 시작 (시간 0 = 사용안함 단계는 건너뜀)
void fcStepStart(uint8_t step)
{
	uint8_t m = bMode - 1;

	while(step < STEP_MAX && bPtTime[m][bPt-1][step] == 0)	step++;

	if(step >= STEP_MAX){
		fcRunEnd();
		return;
	}

	bStep      = step;
	iSetTemp   = iPtTemp[m][bPt-1][step] * 10;
	wRemainSec = (uint16_t)bPtTime[m][bPt-1][step] * 60;
	wRampSec   = 0;
	bHoldFg    = 0;

	lPidSum   = 0;
	iPidPvOld = TEMP_ERR;

	bOpenChkDone = 0;
	wOpenCnt = 0;
}

int16_t fcPid(int16_t sv, int16_t pv, int16_t kp, int16_t ki, int16_t kd, int16_t outMin, int16_t outMax)
{
	int32_t e, d, out, lim;

	e = (int32_t)sv - pv;
	d = (iPidPvOld == TEMP_ERR) ? 0 : ((int32_t)pv - iPidPvOld);
	iPidPvOld = pv;

	out = ((int32_t)kp * e) / 10 + ((int32_t)ki * lPidSum) / 100 - (int32_t)kd * d;

	//적분 : 오차 범위 이내 + 출력 포화 방향이 아닐때만 (anti-windup)
	if(e > -PID_I_BAND && e < PID_I_BAND){
		if( !((out >= outMax && e > 0) || (out <= outMin && e < 0)) ){
			lPidSum += e;
		}
	}

	if(ki > 0){
		lim = (int32_t)outMax * 100 / ki;
		if(lPidSum > lim)	lPidSum = lim;
		lim = (int32_t)outMin * 100 / ki;
		if(lPidSum < lim)	lPidSum = lim;
	}

	out = ((int32_t)kp * e) / 10 + ((int32_t)ki * lPidSum) / 100 - (int32_t)kd * d;

	if(out > outMax)	out = outMax;
	if(out < outMin)	out = outMin;

	return (int16_t)out;
}

//1sec : 출력 90% 이상인데 온도 변화 없으면 단선
void fcOpenCheck(int16_t pv)
{
	int16_t delta;
	uint16_t limit;

	if(bOpenChkDone)	return;

	if(bMode == MODE_PTC){
		limit = PTC_OPEN_TIME;
		if(iOut < OPEN_CHK_OUT){ wOpenCnt = 0; return; }
	}
	else{
		limit = PELT_OPEN_TIME;
		if( !((iOut >=  OPEN_CHK_OUT && bPeltDir == PELT_DIR_HEAT) ||
		      (iOut <= -OPEN_CHK_OUT && bPeltDir == PELT_DIR_COOL)) ){
			wOpenCnt = 0;
			return;
		}
	}

	if(wOpenCnt == 0)	iOpenPv = pv;
	wOpenCnt++;

	if(bMode == MODE_PELT && bPeltDir == PELT_DIR_COOL)	delta = iOpenPv - pv;
	else												delta = pv - iOpenPv;

	if(delta >= OPEN_CHK_DELTA){
		bOpenChkDone = 1;	//정상 확인 -> 이번 단계 검사 완료
	}
	else if(wOpenCnt >= limit){
		fcSetError( (bMode == MODE_PTC) ? ERR_PTC_OPEN : ERR_PELT_OPEN );
	}
}

//1sec
void fcRun(void)
{
	int16_t pv, e;

	if(bRunState != ST_RUN){
		bPtcDuty = 0;
		iPeltTarget = 0;
		iOut = 0;
		return;
	}

	pv = (bMode == MODE_PTC) ? iTempPtc : iTempPelt;

	lRunSec++;

	if(pv == TEMP_ERR){		//센서 에러는 fcProtectCheck 에서 처리
		bPtcDuty = 0;
		iPeltTarget = 0;
		return;
	}

	//유지시간 카운트 : 설정온도 도달 후 시작
	if(!bHoldFg){
		e = pv - iSetTemp;
		if(e < 0)	e = -e;

		if(e <= HOLD_IN_BAND)					bHoldFg = 1;
		else if(++wRampSec >= RAMP_TIMEOUT_SEC)	bHoldFg = 1;
	}

	if(bHoldFg){
		if(wRemainSec)	wRemainSec--;

		if(wRemainSec == 0){
			fcStepStart(bStep + 1);
			if(bRunState != ST_RUN)	return;
		}
	}

	//온도 제어
	if(bMode == MODE_PTC){
		iOut = fcPid(iSetTemp, pv, PTC_KP, PTC_KI, PTC_KD, 0, 1000);
		bPtcDuty = (uint8_t)((uint32_t)iOut * PTC_DUTY_MAX / 1000);
	}
	else{
		iOut = fcPid(iSetTemp, pv, PELT_KP, PELT_KI, PELT_KD, -1000, 1000);
		iPeltTarget = iOut;
	}

	fcOpenCheck(pv);
}


//================================================================
// 통신  (HMI <-> MCU)   ':' + 명령 + CR LF
//================================================================
uint8_t fcCmdIs(const char *cmd)
{
	uint8_t i=0;

	while(cmd[i]){
		if(bCmd[i] != cmd[i])	return 0;
		i++;
	}
	if(bCmd[i] == 0 || bCmd[i] == ' ')	return 1;
	return 0;
}

//명령어 뒤 숫자 인자 -> iArg[] , 리턴 = 개수 (0xff = 형식 에러)
uint8_t fcGetArgs(void)
{
	uint8_t i=0, n=0, neg, digit;
	int16_t v;

	while(bCmd[i] && bCmd[i] != ' ')	i++;

	while(bCmd[i])
	{
		while(bCmd[i] == ' ' || bCmd[i] == ',')	i++;
		if(bCmd[i] == 0)	break;
		if(n >= ARG_MAX)	return 0xff;

		neg = 0;
		if(bCmd[i] == '-'){ neg = 1; i++; }

		v = 0;
		digit = 0;
		while(bCmd[i] >= '0' && bCmd[i] <= '9'){
			if(++digit > 4)	return 0xff;
			v = v*10 + (bCmd[i] - 0x30);
			i++;
		}
		if(digit == 0)	return 0xff;
		if(bCmd[i] != 0 && bCmd[i] != ' ' && bCmd[i] != ',')	return 0xff;

		iArg[n++] = neg ? -v : v;
	}
	return n;
}

void Receive(void)
{
	uint8_t i, n, m, p, s;
	uint8_t bTxFg = 0;

	if(!bRxOk)	return;

	cli();
	for(i=0;i<bRxLen;i++){
		bCmd[i] = bRx[i];
		if(bCmd[i] >= 'a' && bCmd[i] <= 'z')	bCmd[i] -= 0x20;	//대문자
	}
	bCmd[i] = 0;
	bRxOk = 0;
	sei();

	n = fcGetArgs();

	if(fcCmdIs("MODE"))		//:MODE m   (0=없음 1=PTC 2=PELT)
	{
		if(n == 1 && iArg[0] >= MODE_NONE && iArg[0] <= MODE_PELT &&
		   bRunState != ST_RUN && bRunState != ST_PAUSE){
			bMode = iArg[0];
			if(bRunState == ST_END)	bRunState = ST_IDLE;
			bTxFg = TX_OK;
		}
		else bTxFg = TX_FAIL;
	}
	else if(fcCmdIs("RUN"))	//:RUN p  또는  :RUN m p
	{
		if(n == 2 && iArg[0] >= MODE_PTC && iArg[0] <= MODE_PELT &&
		   bRunState != ST_RUN && bRunState != ST_PAUSE){
			bMode = iArg[0];
			iArg[0] = iArg[1];
			n = 1;
		}

		if(n == 1 && iArg[0] >= 1 && iArg[0] <= PT_MAX && bMode != MODE_NONE &&
		   bError == ERR_NONE && bRunState != ST_RUN && bRunState != ST_PAUSE){
			fcRunStart(iArg[0]);
			bTxFg = TX_OK;
		}
		else bTxFg = TX_FAIL;
	}
	else if(fcCmdIs("STOP"))
	{
		fcRunStop();
		bTxFg = TX_OK;
	}
	else if(fcCmdIs("PAUSE"))
	{
		if(bRunState == ST_RUN){
			fcAllOff();
			bRunState = ST_PAUSE;
			bTxFg = TX_OK;
		}
		else bTxFg = TX_FAIL;
	}
	else if(fcCmdIs("RESUME"))
	{
		if(bRunState == ST_PAUSE && bError == ERR_NONE){
			bRunState = ST_RUN;
			bHoldFg = 0;		//설정온도 재도달 후 남은시간 카운트
			wRampSec = 0;
			lPidSum = 0;
			iPidPvOld = TEMP_ERR;
			bOpenChkDone = 0;
			wOpenCnt = 0;
			bTxFg = TX_OK;
		}
		else bTxFg = TX_FAIL;
	}
	else if(fcCmdIs("CLR"))	//에러 해제
	{
		if( (iTempPtc  == TEMP_ERR || iTempPtc  < PTC_OVER_TEMP  - 100) &&
		    (iTempPelt == TEMP_ERR || iTempPelt < PELT_OVER_TEMP - 100) ){
			bError = ERR_NONE;
			bFanPtcErrCnt = 0;
			bFanPeltErrCnt = 0;
			bSensorErrCnt = 0;
			fcAllOff();
			bRunState = ST_IDLE;
			bTxFg = TX_OK;
		}
		else bTxFg = TX_FAIL;
	}
	else if(fcCmdIs("SET"))	//:SET m p s temp time
	{
		bTxFg = TX_FAIL;
		if(n == 5 && iArg[0] >= MODE_PTC && iArg[0] <= MODE_PELT &&
		   iArg[1] >= 1 && iArg[1] <= PT_MAX && iArg[2] >= 1 && iArg[2] <= STEP_MAX)
		{
			m = iArg[0];
			p = iArg[1];
			s = iArg[2];

			if( (iArg[4] == 0 || (iArg[4] >= TIME_SET_MIN && iArg[4] <= TIME_SET_MAX)) &&
			    ((m == MODE_PTC  && iArg[3] >= PTC_SET_MIN  && iArg[3] <= PTC_SET_MAX) ||
			     (m == MODE_PELT && iArg[3] >= PELT_SET_MIN && iArg[3] <= PELT_SET_MAX)) )
			{
				iPtTemp[m-1][p-1][s-1] = iArg[3];
				bPtTime[m-1][p-1][s-1] = iArg[4];
				fcDataSave(m-1, p-1, s-1);
				bTxFg = TX_OK;
			}
		}
	}
	else if(fcCmdIs("GET"))	//:GET m p
	{
		if(n == 2 && iArg[0] >= MODE_PTC && iArg[0] <= MODE_PELT && iArg[1] >= 1 && iArg[1] <= PT_MAX){
			bGetMode = iArg[0];
			bGetPt   = iArg[1];
			bTxFg = TX_PT;
		}
		else bTxFg = TX_FAIL;
	}
	else if(fcCmdIs("STATUS?"))
	{
		bTxFg = TX_DAT;
	}
	else
	{
		bTxFg = TX_FAIL;
	}

	if(bTxFg)	Transfer(bTxFg);
}

uint8_t fcTxNum(uint8_t idx, int32_t v)
{
	uint8_t buf[10], n=0;

	if(v < 0){
		TxData[idx++] = '-';
		v = -v;
	}
	do{
		buf[n++] = (v % 10) + 0x30;
		v /= 10;
	}while(v && n < 10);

	while(n)	TxData[idx++] = buf[--n];

	return idx;
}

void Transfer(uint8_t type)
{
	uint8_t bTxlen=0, i;
	int16_t pv, sv;

	TxData[bTxlen++] = ':';

	if(type == TX_DAT)
	{
		//:DAT mode state pt step sv pv remain hold out err elapsed
		pv = (bMode == MODE_PELT) ? iTempPelt : iTempPtc;
		sv = (bRunState == ST_RUN || bRunState == ST_PAUSE) ? iSetTemp : 0;

		TxData[bTxlen++] = 'D';
		TxData[bTxlen++] = 'A';
		TxData[bTxlen++] = 'T';
		TxData[bTxlen++] = ' ';	bTxlen = fcTxNum(bTxlen, bMode);
		TxData[bTxlen++] = ' ';	bTxlen = fcTxNum(bTxlen, bRunState);
		TxData[bTxlen++] = ' ';	bTxlen = fcTxNum(bTxlen, bPt);
		TxData[bTxlen++] = ' ';	bTxlen = fcTxNum(bTxlen, bStep + 1);
		TxData[bTxlen++] = ' ';	bTxlen = fcTxNum(bTxlen, sv);
		TxData[bTxlen++] = ' ';	bTxlen = fcTxNum(bTxlen, pv);
		TxData[bTxlen++] = ' ';	bTxlen = fcTxNum(bTxlen, wRemainSec);
		TxData[bTxlen++] = ' ';	bTxlen = fcTxNum(bTxlen, bHoldFg);
		TxData[bTxlen++] = ' ';	bTxlen = fcTxNum(bTxlen, iOut);
		TxData[bTxlen++] = ' ';	bTxlen = fcTxNum(bTxlen, bError);
		TxData[bTxlen++] = ' ';	bTxlen = fcTxNum(bTxlen, lRunSec);
	}
	else if(type == TX_FAIL)
	{
		TxData[bTxlen++] ='F';
		TxData[bTxlen++] ='A';
		TxData[bTxlen++] ='I';
		TxData[bTxlen++] ='L';
	}
	else if(type == TX_OK)
	{
		TxData[bTxlen++] ='O';
		TxData[bTxlen++] ='K';
	}
	else if(type == TX_PT)
	{
		//:PT m p t1 n1 t2 n2 ... t6 n6
		TxData[bTxlen++] = 'P';
		TxData[bTxlen++] = 'T';
		TxData[bTxlen++] = ' ';	bTxlen = fcTxNum(bTxlen, bGetMode);
		TxData[bTxlen++] = ' ';	bTxlen = fcTxNum(bTxlen, bGetPt);
		for(i=0;i<STEP_MAX;i++){
			TxData[bTxlen++] = ' ';	bTxlen = fcTxNum(bTxlen, iPtTemp[bGetMode-1][bGetPt-1][i]);
			TxData[bTxlen++] = ' ';	bTxlen = fcTxNum(bTxlen, bPtTime[bGetMode-1][bGetPt-1][i]);
		}
	}
	else if(type == TX_END)
	{
		//:END mode pt
		TxData[bTxlen++] = 'E';
		TxData[bTxlen++] = 'N';
		TxData[bTxlen++] = 'D';
		TxData[bTxlen++] = ' ';	bTxlen = fcTxNum(bTxlen, bMode);
		TxData[bTxlen++] = ' ';	bTxlen = fcTxNum(bTxlen, bPt);
	}
	else if(type == TX_ERR)
	{
		//:ERR n
		TxData[bTxlen++] = 'E';
		TxData[bTxlen++] = 'R';
		TxData[bTxlen++] = 'R';
		TxData[bTxlen++] = ' ';	bTxlen = fcTxNum(bTxlen, bError);
	}

	TxData[bTxlen++] = 0x0d;
	TxData[bTxlen++] = 0x0a;

	UART_SendString1(&TxData[0],bTxlen);
}

void UART_Tx(uint8_t ch)
{
	while (! (UCSR1A & (1<<UDRE1)));	/* Wait for empty transmit buffer*/
	UDR1 = ch ;
}

void UART0_Tx(uint8_t ch)
{
	while (! (UCSR0A & (1<<UDRE0)));
	UDR0 = ch ;
}

void UART_SendString1(uint8_t *str, uint8_t len)
{
	uint8_t j=0;

	for(j=0 ; j<len ;j++)
	{
		UART_Tx(str[j]);
#if DEBUG_TX_USE
		UART0_Tx(str[j]);
#endif
	}
}


ISR(USART1_RX_vect)
{
	rxbuf = UDR1;

	switch(bRxSeq){
		case 0:
			if(rxbuf == ':') {//
				bRxCnt=0;
				bRxSeq=1;
			}
			else bRxCnt=0;
		break;
		case 1:

			if(rxbuf==0x0d ){
				bRxSeq=2;
			}
			else if(rxbuf == ':') {
				bRxCnt=0;
				bRxSeq=1;
			}
			else{
				bRxData[bRxCnt] = rxbuf;
				if(++bRxCnt>=RX_BUF_MAX)	bRxCnt=0;
			}

		break;
		case 2:

			if(rxbuf == 0x0a) {//
				if(!bRxOk){		//이전 명령 처리 완료 후 갱신
					for(len=0;len<bRxCnt;len++){
						bRx[len] = bRxData[len] ;
					}
					bRxLen = bRxCnt;
					bRxOk=1;
				}
				bRxSeq=0;
				bRxCnt=0;
			}
			else if(rxbuf == ':') {
				bRxCnt=0;
				bRxSeq=1;
			}
			else{
				bRxSeq=0;
				bRxCnt=0;
			}

		break;
	}
}


//================================================================
// EEPROM
//================================================================
uint16_t fcPtAddr(uint8_t m, uint8_t p, uint8_t s)
{
	return ADDR_PT + ((uint16_t)(m*PT_MAX + p)*STEP_MAX + s) * 3;
}

void fcDataErr_Check(void)
{
	uint8_t m, p, s, err;
	int16_t tmin, tmax;

	for(m=0;m<2;m++){
		tmin = (m == 0) ? PTC_SET_MIN : PELT_SET_MIN;
		tmax = (m == 0) ? PTC_SET_MAX : PELT_SET_MAX;

		for(p=0;p<PT_MAX;p++){
			for(s=0;s<STEP_MAX;s++){
				err = 0;
				if(iPtTemp[m][p][s] < tmin || iPtTemp[m][p][s] > tmax)	err = 1;
				if(bPtTime[m][p][s] > TIME_SET_MAX)						err = 1;

				if(err){
					iPtTemp[m][p][s] = iDefTemp[m][s];
					bPtTime[m][p][s] = bDefTime[m][s];
					fcDataSave(m, p, s);
				}
			}
		}
	}
}

void fcDataSave(uint8_t mode, uint8_t pt, uint8_t step)
{
	uint16_t addr = fcPtAddr(mode, pt, step);

	EEPROM_WriteByte(addr,   (iPtTemp[mode][pt][step] >> 8) & 0xff);
	EEPROM_WriteByte(addr+1,  iPtTemp[mode][pt][step] & 0xff);
	EEPROM_WriteByte(addr+2,  bPtTime[mode][pt][step]);
	wdt_reset();
}

void fcReadData(void)
{
	uint16_t addr_temp;
	uint8_t m, p, s;

	addr_temp = EEPROM_ReadByte(ADDR_INIT_H);
	addr_temp <<= 8;
	addr_temp |= EEPROM_ReadByte(ADDR_INIT_L);

	if(addr_temp!=INIT_DATA)
	{
		for(m=0;m<2;m++){
			for(p=0;p<PT_MAX;p++){
				for(s=0;s<STEP_MAX;s++){
					iPtTemp[m][p][s] = iDefTemp[m][s];
					bPtTime[m][p][s] = bDefTime[m][s];
					fcDataSave(m, p, s);
				}
			}
		}

		addr_temp=INIT_DATA;
		EEPROM_WriteByte(ADDR_INIT_H,((addr_temp>>8)&0xff));
		EEPROM_WriteByte(ADDR_INIT_L,(addr_temp&0xff));
	}

	for(m=0;m<2;m++){
		for(p=0;p<PT_MAX;p++){
			for(s=0;s<STEP_MAX;s++){
				addr_temp = fcPtAddr(m, p, s);
				iPtTemp[m][p][s] = (int16_t)(((uint16_t)EEPROM_ReadByte(addr_temp) << 8) | EEPROM_ReadByte(addr_temp+1));
				bPtTime[m][p][s] = EEPROM_ReadByte(addr_temp+2);
			}
		}
	}

	fcDataErr_Check();
}

void EEPROM_WriteByte(uint16_t Address, uint8_t Data)
{
	uint8_t sreg;

	while(EECR & (1<<EEWE));

	sreg = SREG;
	cli();

	EEAR = Address;
	EEDR = Data;

	EECR |=(1<<EEMWE);
	EECR |=(1<<EEWE);

	SREG = sreg;
}


uint8_t EEPROM_ReadByte(uint16_t Address)
{
	while(EECR & (1<<EEWE));

	EEAR = Address;
	EECR |=(1<<EERE);

	return EEDR;
}
