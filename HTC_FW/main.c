
/*
 * HTC main.c : PTC / Peltier 6단계 온도 프로토콜 제어
 * MCU : ATmega128A-AU 16MHz  (VSHCCB_V1)
 */
//261030
//
// Fuse : M103C 해제(필수), JTAGEN 해제(필수 : PF4/PF5 FAN_ADC 사용), BODEN 4.0V, 외부 16MHz 크리스탈
// NTC 온도는 테이블 보간으로 계산 (math.h / libm 불필요)

#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
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

//protocol 저장값 [mode][pt][step] (temp 0.1℃)
int16_t iPtTemp[2][PT_MAX][STEP_MAX];
uint8_t bPtTime[2][PT_MAX][STEP_MAX];

//운전중 레시피 (RUN 시점의 화면 값 복사)
int16_t iRunTemp[STEP_MAX];
uint8_t bRunTime[STEP_MAX];

//eeprom 비동기 저장
uint8_t  bEepBusy=0;
uint16_t wEepIdx=0;

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

//modbus
uint16_t wMbReg[MB_W_REG_NUM]={0,};		//쓰기 영역 0~21
uint8_t  bMbCmdFg=0;
volatile uint8_t bMbRxBuf[MB_BUF_MAX];
volatile uint8_t bMbRxCnt=0, bMbRxErr=0;
volatile uint8_t bMbRxActive=0, bMbGapCnt=0, bMbFrameFg=0;
volatile uint8_t bMbTxBuf[MB_BUF_MAX];
volatile uint8_t bMbTxLen=0, bMbTxIdx=0;

//debug
uint8_t TxData[80];


ISR(TIMER2_COMP_vect)   //1ms
{
	//modbus 프레임 끝 (3.5문자 무신호)
	if(bMbRxActive){
		if(++bMbGapCnt >= MB_FRAME_GAP){
			bMbRxActive = 0;
			if(bMbRxCnt)	bMbFrameFg = 1;
		}
	}

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

//USART1 : TOUCH_COM(HMI Modbus 9600) , USART0 : DEBUG_COM(19200)
void UART_init(void)
{
	UCSR1A = (1 << U2X1); // x2
	UCSR1B = (1 << RXCIE1) | (1 << RXEN1) | (1 << TXEN1);
	UCSR1C = (1 << UCSZ11) | (1 << UCSZ10); // 8bit, 1stop, no parity
	UBRR1H = 0;
	UBRR1L = UBRR_9600;

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


//NTC 50K B3950 저항표 (Ω) : -30℃ ~ 160℃, 5℃ 간격
//  센서 B 정수가 다르면 R = 50000 * exp(B*(1/(T+273.15) - 1/298.15)) 로 다시 계산
const uint32_t lNtcTbl[NTC_TBL_NUM] PROGMEM = {
	1001020, 721585, 526923, 389491, 291229, 220130, 168103, 129623,	// -30 ~   5
	 100873,  79186,  62677,  50000,  40186,  32528,  26507,  21741,	//  10 ~  45
	  17941,  14892,  12431,  10432,   8799,   7458,   6352,   5433,	//  50 ~  85
	   4668,   4027,   3488,   3032,   2646,   2317,   2035,   1794,	//  90 ~ 125
	   1586,   1407,   1252,   1116,    998,    895,    805			// 130 ~ 160
};

//NTC 50K : ADC -> 0.1℃ (저항 계산 후 표에서 직선 보간)
int16_t fcNtcTemp(uint16_t adc)
{
	uint32_t r, r_hi, r_lo;
	uint8_t i;

	if(adc < NTC_ADC_OPEN || adc > NTC_ADC_SHORT)	return TEMP_ERR;

	r = (uint32_t)NTC_R_PULLDOWN * (1024 - adc) / adc;	//NTC 저항 Ω

	if(r >= pgm_read_dword(&lNtcTbl[0]))				return NTC_TBL_T0 * 10;
	if(r <= pgm_read_dword(&lNtcTbl[NTC_TBL_NUM-1]))	return (NTC_TBL_T0 + (NTC_TBL_NUM-1)*NTC_TBL_STEP) * 10;

	for(i=0;i<NTC_TBL_NUM-1;i++){
		r_hi = pgm_read_dword(&lNtcTbl[i]);
		r_lo = pgm_read_dword(&lNtcTbl[i+1]);

		if(r <= r_hi && r > r_lo){
			return (NTC_TBL_T0 + i*NTC_TBL_STEP) * 10
			     + (int16_t)( (r_hi - r) * (NTC_TBL_STEP*10) / (r_hi - r_lo) );
		}
	}
	return TEMP_ERR;
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

		if(bMbFrameFg)		//HMI 요청 -> 100ms 이내 응답
		{
			fcModbusProcess();

			bMbRxCnt = 0;
			bMbRxErr = 0;
			bMbFrameFg = 0;

			if(bMbCmdFg){
				bMbCmdFg = 0;
				fcCmdProcess();
			}
		}

		fcEepromTask();

		if( b10msFg )
		{
			b10msFg=0;
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

			fcErrorClearCheck();

			Transfer(TX_DAT);	//DEBUG 1sec
		}
	}
}


//================================================================
// 프로토콜 실행
//================================================================
//bMode, bPt, iRunTemp[], bRunTime[] 설정 후 호출
void fcRunStart(void)
{
	fcAllOff();
	lRunSec = 0;
	bFanPtcReq = 0;
	bRunState = ST_RUN;

	fcStepStart(0);
}

//1sec : 원인이 해소된 에러는 자동 해제 -> ERROR=0, STATE=대기
//       (FAN / 단선 에러는 동작해야 확인 가능하므로 다음 RUN 명령에서 해제)
void fcErrorClearCheck(void)
{
	static uint8_t bOkCnt = 0;
	uint8_t ok = 0;

	if(bError == ERR_NONE){ bOkCnt = 0; return; }

	switch(bError){
		case ERR_PTC_OVER:
			ok = (iTempPtc != TEMP_ERR && iTempPtc < PTC_OVER_TEMP - 100);	//10℃ 히스테리시스
		break;
		case ERR_PELT_OVER:
			ok = (iTempPelt != TEMP_ERR && iTempPelt < PELT_OVER_TEMP - 100);
#if PELT_HS_SENSOR_USE
			if(iTempHs == TEMP_ERR || iTempHs >= PELT_HS_OVER_TEMP - 100)	ok = 0;
#endif
		break;
		case ERR_PTC_SENSOR:
			ok = (iTempPtc != TEMP_ERR);
		break;
		case ERR_PELT_SENSOR:
			ok = (iTempPelt != TEMP_ERR);
		break;
		default:
			ok = 0;
		break;
	}

	if(ok){
		if(++bOkCnt >= 5){		//5초 연속 정상
			bOkCnt = 0;
			bError = ERR_NONE;
			bSensorErrCnt = 0;
			bRunState = ST_IDLE;
		}
	}
	else bOkCnt = 0;
}

void fcRunEnd(void)
{
	fcAllOff();
	bRunState = ST_END;
	wRemainSec = 0;

	Transfer(TX_END);	//DEBUG
}

//다음 유효 단계 시작 (시간 0 = 사용안함 단계는 건너뜀)
void fcStepStart(uint8_t step)
{
	while(step < STEP_MAX && bRunTime[step] == 0)	step++;

	if(step >= STEP_MAX){
		fcRunEnd();
		return;
	}

	bStep      = step;
	iSetTemp   = iRunTemp[step];
	wRemainSec = (uint16_t)bRunTime[step] * 60;
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
			fcStepStart(bStep + 1);		//STEP +1, 6단계 끝나면 STATE=완료
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
// 통신  HMI(Autobase, 마스터) <-> MCU(슬레이브 1) : Modbus RTU 9600 8N1
//  FC03 읽기 : 0~21(쓰기 영역), 100~104(STATE, STEP, ELAPSED, PT_TEMP, ERROR)
//  FC06/FC16 쓰기 : 0~21  -> CMD(2) 에 쓰기가 들어오면 명령 처리 후 0 으로
//================================================================
uint16_t fcMbCrc(uint8_t *buf, uint8_t len)
{
	uint16_t crc = 0xffff;
	uint8_t i, j;

	for(i=0;i<len;i++){
		crc ^= buf[i];
		for(j=0;j<8;j++){
			if(crc & 0x0001)	crc = (crc >> 1) ^ 0xa001;
			else				crc >>= 1;
		}
	}
	return crc;
}

//읽기 레지스터 값 (0xffff 리턴용 플래그 대신 ok 포인터)
uint8_t fcMbRead(uint16_t addr, uint16_t *val)
{
	int16_t pv;
	uint8_t err;

	if(addr < MB_W_REG_NUM){
		*val = wMbReg[addr];
		return 1;
	}

	switch(addr){
		case REG_R_START:		//STATE
			*val = bRunState;
		break;
		case REG_R_START+1:		//STEP 1~6
			*val = bStep + 1;
		break;
		case REG_R_START+2:		//ELAPSED sec
			*val = (lRunSec > 65535) ? 65535 : (uint16_t)lRunSec;
		break;
		case REG_R_START+3:		//PT_TEMP x10
			if(bRunState == ST_IDLE)	pv = (wMbReg[REG_MODE] == MODE_PELT) ? iTempPelt : iTempPtc;
			else						pv = (bMode == MODE_PELT) ? iTempPelt : iTempPtc;
			if(pv == TEMP_ERR)	pv = 0;
			*val = (uint16_t)pv;
		break;
		case REG_R_START+4:		//ERROR 1~6
			err = bError;
			if(err == ERR_PTC_SENSOR)	err = ERR_PTC_OPEN;
			if(err == ERR_PELT_SENSOR)	err = ERR_PELT_OPEN;
			*val = err;
		break;
		default:
			return 0;
	}
	return 1;
}

void fcMbWrite(uint16_t addr, uint16_t val)
{
	wMbReg[addr] = val;
	if(addr == REG_CMD)	bMbCmdFg = 1;	//같은 값이라도 쓰기 때마다 명령 처리
}

void fcMbSend(uint8_t len)
{
	uint16_t crc;

	crc = fcMbCrc((uint8_t *)bMbTxBuf, len);
	bMbTxBuf[len++] = crc & 0xff;		//CRC Low 먼저
	bMbTxBuf[len++] = crc >> 8;

	while(UCSR1B & (1<<UDRIE1));		//이전 송신 완료 대기

	bMbTxLen = len;
	bMbTxIdx = 0;
	UCSR1B |= (1<<UDRIE1);				//인터럽트 송신 시작
}

void fcMbException(uint8_t func, uint8_t code)
{
	bMbTxBuf[0] = MB_SLAVE_ID;
	bMbTxBuf[1] = func | 0x80;
	bMbTxBuf[2] = code;
	fcMbSend(3);
}

void fcModbusProcess(void)
{
	uint8_t  *rx = (uint8_t *)bMbRxBuf;
	uint8_t  n = bMbRxCnt, i, func, bcast, txlen;
	uint16_t addr, qty, val, crc;

	if(bMbRxErr || n < 8)	return;

	crc = fcMbCrc(rx, n-2);
	if( (crc & 0xff) != rx[n-2] || (crc >> 8) != rx[n-1] )	return;	//CRC 에러 -> 무응답

	if(rx[0] != MB_SLAVE_ID && rx[0] != 0)	return;	//다른 주소
	bcast = (rx[0] == 0);

	func = rx[1];
	addr = ((uint16_t)rx[2] << 8) | rx[3];
	qty  = ((uint16_t)rx[4] << 8) | rx[5];

	if(func == 0x03)		//Read Holding Registers
	{
		if(bcast)	return;
		if(qty == 0 || qty > (MB_BUF_MAX-5)/2){ fcMbException(func, 0x03); return; }

		bMbTxBuf[0] = MB_SLAVE_ID;
		bMbTxBuf[1] = func;
		bMbTxBuf[2] = qty * 2;
		txlen = 3;

		for(i=0;i<qty;i++){
			if(!fcMbRead(addr + i, &val)){ fcMbException(func, 0x02); return; }
			bMbTxBuf[txlen++] = val >> 8;
			bMbTxBuf[txlen++] = val & 0xff;
		}
		fcMbSend(txlen);
	}
	else if(func == 0x06)	//Write Single Register
	{
		if(addr >= MB_W_REG_NUM){ if(!bcast) fcMbException(func, 0x02); return; }

		fcMbWrite(addr, qty);	//qty 위치 = 값

		if(bcast)	return;
		for(i=0;i<6;i++)	bMbTxBuf[i] = rx[i];	//요청 그대로 응답
		fcMbSend(6);
	}
	else if(func == 0x10)	//Write Multiple Registers
	{
		if(n < 9 || qty == 0 || rx[6] != qty*2 || n != 9 + qty*2){ if(!bcast) fcMbException(func, 0x03); return; }
		if(addr + qty > MB_W_REG_NUM){ if(!bcast) fcMbException(func, 0x02); return; }

		for(i=0;i<qty;i++){
			val = ((uint16_t)rx[7 + i*2] << 8) | rx[8 + i*2];
			fcMbWrite(addr + i, val);
		}

		if(bcast)	return;
		for(i=0;i<6;i++)	bMbTxBuf[i] = rx[i];
		fcMbSend(6);
	}
	else
	{
		if(!bcast)	fcMbException(func, 0x01);
	}
}

//레시피 레지스터(10~21) -> 운전 테이블. 범위 밖이면 0 리턴
uint8_t fcRecipeLoad(uint8_t mode, int16_t *temp, uint8_t *time)
{
	uint8_t i;
	int16_t t;
	uint16_t m;

	for(i=0;i<STEP_MAX;i++){
		t = (int16_t)wMbReg[REG_RECIPE + i*2];
		m = wMbReg[REG_RECIPE + i*2 + 1];

		if(m > TIME_SET_MAX)	return 0;

		if(mode == MODE_PTC){
			if(t < PTC_SET_MIN || t > PTC_SET_MAX)	return 0;
		}
		else{
			if(t < PTC_SET_MIN || t > PTC_SET_MAX)	return 0;	//화면 범위 20.0~120.0
			if(t > PELT_SET_MAX)	t = PELT_SET_MAX;			//펠티어 보호 제한
			if(t < PELT_SET_MIN)	t = PELT_SET_MIN;
		}
		temp[i] = t;
		time[i] = (uint8_t)m;
	}
	return 1;
}

void fcCmdProcess(void)
{
	uint16_t cmd, mode, pt;
	uint8_t  i;
	int16_t  temp[STEP_MAX];
	uint8_t  time[STEP_MAX];

	cmd = wMbReg[REG_CMD];
	wMbReg[REG_CMD] = CMD_NONE;		//처리 후 0 으로

	mode = wMbReg[REG_MODE];
	pt   = wMbReg[REG_PT_NO];

	switch(cmd){
		case CMD_RUN:
			if(bRunState == ST_RUN)	break;		//운전중 RUN 은 무시
			if(mode < MODE_PTC || mode > MODE_PELT || pt < 1 || pt > PT_MAX)	break;
			if(!fcRecipeLoad(mode, temp, time))	break;

			//FAN / 단선 에러는 RUN 시 해제 후 재확인 (과열/센서는 즉시 다시 감지됨)
			if(bError){
				bError = ERR_NONE;
				bFanPtcErrCnt = 0;
				bFanPeltErrCnt = 0;
				bSensorErrCnt = 0;
			}

			bMode = mode;
			bPt   = pt;
			for(i=0;i<STEP_MAX;i++){
				iRunTemp[i] = temp[i];
				bRunTime[i] = time[i];
			}
			fcRunStart();
		break;

		case CMD_PAUSE:		//일시정지 : STEP, ELAPSED 유지
			if(bRunState == ST_RUN){
				fcAllOff();
				bRunState = ST_PAUSE;
			}
		break;

		case CMD_STOP:		//정지 : 출력 OFF -> 대기 (에러 상태는 유지)
			if(bRunState == ST_RUN || bRunState == ST_PAUSE || bRunState == ST_END){
				fcAllOff();
				bRunState = ST_IDLE;
				wRemainSec = 0;
			}
		break;

		case CMD_RESUME:
			if(bRunState == ST_PAUSE && bError == ERR_NONE){
				bRunState = ST_RUN;
				bHoldFg = 0;		//설정온도 재도달 후 남은시간 카운트
				wRampSec = 0;
				lPidSum = 0;
				iPidPvOld = TEMP_ERR;
				bOpenChkDone = 0;
				wOpenCnt = 0;
			}
		break;

		case CMD_SAVE:
			pt = wMbReg[REG_SAVE_PT];
			if(mode < MODE_PTC || mode > MODE_PELT || pt < 1 || pt > PT_MAX)	break;
			if(!fcRecipeLoad(MODE_PTC, temp, time))	break;	//화면 값 그대로 저장 (20.0~120.0)

			for(i=0;i<STEP_MAX;i++){
				iPtTemp[mode-1][pt-1][i] = temp[i];
				bPtTime[mode-1][pt-1][i] = time[i];
			}
			bEepBusy = 1;		//EEPROM 비동기 저장 (응답 지연 방지)
			wEepIdx = 0;
		break;
	}
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

//DEBUG_COM 출력 (USART0 19200)
void Transfer(uint8_t type)
{
#if DEBUG_TX_USE
	uint8_t bTxlen=0;
	int16_t pv;

	TxData[bTxlen++] = ':';

	if(type == TX_DAT)
	{
		//:DAT mode state pt step sv pv remain hold out err elapsed
		pv = (bMode == MODE_PELT) ? iTempPelt : iTempPtc;

		TxData[bTxlen++] = 'D';
		TxData[bTxlen++] = 'A';
		TxData[bTxlen++] = 'T';
		TxData[bTxlen++] = ' ';	bTxlen = fcTxNum(bTxlen, bMode);
		TxData[bTxlen++] = ' ';	bTxlen = fcTxNum(bTxlen, bRunState);
		TxData[bTxlen++] = ' ';	bTxlen = fcTxNum(bTxlen, bPt);
		TxData[bTxlen++] = ' ';	bTxlen = fcTxNum(bTxlen, bStep + 1);
		TxData[bTxlen++] = ' ';	bTxlen = fcTxNum(bTxlen, iSetTemp);
		TxData[bTxlen++] = ' ';	bTxlen = fcTxNum(bTxlen, pv);
		TxData[bTxlen++] = ' ';	bTxlen = fcTxNum(bTxlen, wRemainSec);
		TxData[bTxlen++] = ' ';	bTxlen = fcTxNum(bTxlen, bHoldFg);
		TxData[bTxlen++] = ' ';	bTxlen = fcTxNum(bTxlen, iOut);
		TxData[bTxlen++] = ' ';	bTxlen = fcTxNum(bTxlen, bError);
		TxData[bTxlen++] = ' ';	bTxlen = fcTxNum(bTxlen, lRunSec);
	}
	else if(type == TX_END)
	{
		TxData[bTxlen++] = 'E';
		TxData[bTxlen++] = 'N';
		TxData[bTxlen++] = 'D';
		TxData[bTxlen++] = ' ';	bTxlen = fcTxNum(bTxlen, bMode);
		TxData[bTxlen++] = ' ';	bTxlen = fcTxNum(bTxlen, bPt);
	}
	else if(type == TX_ERR)
	{
		TxData[bTxlen++] = 'E';
		TxData[bTxlen++] = 'R';
		TxData[bTxlen++] = 'R';
		TxData[bTxlen++] = ' ';	bTxlen = fcTxNum(bTxlen, bError);
	}

	TxData[bTxlen++] = 0x0d;
	TxData[bTxlen++] = 0x0a;

	UART_SendString0(&TxData[0],bTxlen);
#else
	(void)type;
#endif
}

void UART0_Tx(uint8_t ch)
{
	while (! (UCSR0A & (1<<UDRE0)));
	UDR0 = ch ;
}

void UART_SendString0(uint8_t *str, uint8_t len)
{
	uint8_t j=0;

	for(j=0 ; j<len ;j++)	UART0_Tx(str[j]);
}

//modbus 송신 (인터럽트)
ISR(USART1_UDRE_vect)
{
	if(bMbTxIdx < bMbTxLen){
		UDR1 = bMbTxBuf[bMbTxIdx++];
	}
	else{
		UCSR1B &= ~(1<<UDRIE1);
	}
}

//modbus 수신 : 바이트 저장, 무신호 시간은 1ms 타이머에서 판단
ISR(USART1_RX_vect)
{
	uint8_t st, rxbuf;

	st = UCSR1A;
	rxbuf = UDR1;

	if(bMbFrameFg)	return;		//이전 프레임 처리중

	if(st & ((1<<FE1)|(1<<DOR1)))	bMbRxErr = 1;

	if(bMbRxCnt < MB_BUF_MAX)	bMbRxBuf[bMbRxCnt++] = rxbuf;
	else						bMbRxErr = 1;

	bMbGapCnt = 0;
	bMbRxActive = 1;
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
	uint8_t m, p, s;

	for(m=0;m<2;m++){
		for(p=0;p<PT_MAX;p++){
			for(s=0;s<STEP_MAX;s++){
				if(iPtTemp[m][p][s] < PTC_SET_MIN || iPtTemp[m][p][s] > PTC_SET_MAX ||
				   bPtTime[m][p][s] > TIME_SET_MAX)
				{
					iPtTemp[m][p][s] = DEF_TEMP;
					bPtTime[m][p][s] = DEF_TIME;
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
					iPtTemp[m][p][s] = DEF_TEMP;
					bPtTime[m][p][s] = DEF_TIME;
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

//main loop : SAVE 값을 EEPROM 에 1byte 씩 저장 (변경된 byte 만, 대기 없이)
void fcEepromTask(void)
{
	uint8_t m, p, s, part, val;
	uint16_t i;

	if(!bEepBusy)				return;
	if(EECR & (1<<EEWE))		return;		//이전 쓰기 진행중

	while(wEepIdx < EEP_PT_SIZE)
	{
		i    = wEepIdx / 3;
		part = wEepIdx % 3;
		s = i % STEP_MAX;
		p = (i / STEP_MAX) % PT_MAX;
		m = i / (STEP_MAX * PT_MAX);

		if(part == 0)		val = (iPtTemp[m][p][s] >> 8) & 0xff;
		else if(part == 1)	val =  iPtTemp[m][p][s] & 0xff;
		else				val =  bPtTime[m][p][s];

		wEepIdx++;

		if(EEPROM_ReadByte(ADDR_PT + wEepIdx - 1) != val){
			EEPROM_WriteByte(ADDR_PT + wEepIdx - 1, val);
			return;							//다음 loop 에서 계속
		}
	}
	bEepBusy = 0;
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
