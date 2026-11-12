/*
 * SVROCB.c
 *
 * 로드셀 컨트롤 보드 (SVROCB_V1, ATmega8A-AU 16MHz)
 *  - 로드셀(100g) 5채널 : INA155(x50) -> MCP6242(x11) -> ADC0~4
 *  - 무게 계산(0.1g) / 감지시 스테이션 LED(1ST~5ST) ON
 *  - DIP S/W 로 보드 ID 설정 (ID0 = Load Cell 1~5, ID1 = Load Cell 6~10)
 *  - RS485 (Modbus RTU 슬레이브) 로 메인보드에 무게 / 상태 송신
 */
//261112
//최초 작성

#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include "define.h"

void Port_Init(void);
void Timer1_Init(void);
void ADC_Init(void);
void UART_init(void);
void fcLoadCell(void);
void fcStLed(uint8_t num, uint8_t on);
void fcStLedControl(void);
void fcStateLed(void);
void fcIdCheck(void);
void fcPowerOnZero(void);
void fcCmdProcess(void);
void fcModbusProcess(void);
void fcSaveData(void);
void fcReadData(void);
void fcCalInit(void);
void EEPROM_WriteByte(uint16_t Address, uint8_t Data);
uint8_t EEPROM_ReadByte(uint16_t Address);

//timer
volatile uint8_t b2msCnt=0;
volatile uint8_t b10msCnt=0,b10msFg=0;
volatile uint8_t b100msCnt=0,b100msFg=0;
uint8_t b1SecCnt=0;
uint8_t bLedCnt=0,bLedFg=0;
uint8_t bBlinkCnt=0,bBlinkFg=0;

//adc
volatile uint16_t AdcData[CH_NUM]={0,};
volatile uint16_t lADC_Buf[CH_NUM]={0,};
volatile uint8_t bAdcCnt=0;
volatile uint8_t bAdcFg=0;
volatile uint8_t bAdcBusy=0;

//load cell
uint16_t wRawBuf[CH_NUM][FILTER_NUM];
uint8_t  bFilterIdx=0,bFilterFull=0;
uint16_t wRaw[CH_NUM]={0,};			//이동평균 raw
uint16_t wZero[CH_NUM]={0,};		//영점 raw (0 = 미설정)
int32_t  lSpan[CH_NUM]={0,};		//100.0g 일때 raw 변화량
int16_t  iWeight[CH_NUM]={0,};		//무게 0.1g
uint8_t  bErrCnt[CH_NUM]={0,};
uint8_t  bSensorErr=0;				//bit0~4 센서 이상
uint8_t  bOverFg=0;					//bit0~4 과부하
uint8_t  bDetect=0;					//bit0~4 감지

//setting
uint16_t wDetectOn=DEF_DETECT_ON,wDetectOff=DEF_DETECT_OFF;
uint16_t wCalWeight=DEF_CAL_WEIGHT;
uint8_t  bLedMode=LED_MODE_AUTO;
uint8_t  bLedManual=0;
uint8_t  bSaveFg=0;
uint8_t  bCmd=CMD_NONE,bCmdFg=0;
uint8_t  bPowerOnCnt=0,bPowerOnFg=0;

//id
uint8_t bBoardId=0;
uint8_t bSlaveAddr=MB_ADDR_BASE;

//uart / modbus
volatile uint8_t bRxBuf[MB_RX_SIZE];
volatile uint8_t bRxLen=0,bRxErr=0;
volatile uint8_t bRxIdleCnt=0,bRxBusy=0,bRxDone=0;
volatile uint8_t bTxBuf[MB_TX_SIZE];
volatile uint8_t bTxLen=0,bTxIdx=0;
volatile uint8_t bTxDelayCnt=0,bTxReq=0;
uint16_t MDregister[MB_REG_NUM]={0,};
uint8_t  bCommCnt=COMM_TIMEOUT,bCommOk=0;


ISR(TIMER1_COMPA_vect)	//1ms
{
	//2ms 마다 ADC0~4 연속 변환 시작
	if(++b2msCnt >= 2)
	{
		b2msCnt = 0;

		if(bAdcBusy==0)
		{
			bAdcBusy = 1;
			ADMUX = 0x00;					//ADC0 , AREF(DC5V) 외부 기준
			ADCSRA |= 1<<ADSC;				//A/D변환 시작
		}
	}

	if(++b10msCnt >= 10)//10ms timer
	{
		b10msCnt = 0;
		b10msFg = 1;

		if(++b100msCnt >= 10){
			b100msCnt = 0;
			b100msFg = 1;
		}
	}

	//modbus 프레임 끝 검출 : MB_FRAME_GAP ms 동안 수신 없음
	if(bRxBusy)
	{
		if(++bRxIdleCnt >= MB_FRAME_GAP)
		{
			bRxBusy = 0;
			bRxDone = 1;
		}
	}

	//응답 지연 후 송신 시작
	if(bTxReq)
	{
		if(++bTxDelayCnt >= MB_TX_DELAY)
		{
			bTxReq = 0;
			bTxIdx = 0;
			RS485_TX_MODE;
			UCSRB |= (1<<UDRIE);			//인터럽트 송신 시작
		}
	}
}

ISR(ADC_vect)
{
	uint8_t ch,k;

	ch = ADMUX & 0x07;
	AdcData[ch] += (ADC & 0x03ff);

	if(ch < (CH_NUM-1))
	{
		ADMUX = ch + 1;
		ADCSRA |= 1<<ADSC;					//다음 채널 A/D 변환 시작
	}
	else //ch4 까지 완료
	{
		bAdcBusy = 0;

		if(++bAdcCnt >= ADC_AVG_CNT)		//2ms x 50 = 100ms
		{
			bAdcCnt = 0;

			for(k=0;k<CH_NUM;k++)
			{
				lADC_Buf[k] = AdcData[k];
				AdcData[k] = 0;
			}
			bAdcFg = 1;
		}
	}
}

ISR(USART_RXC_vect)
{
	uint8_t st,data;

	st = UCSRA;
	data = UDR;

	if(bRxDone || bTxReq || (UCSRB & ((1<<UDRIE)|(1<<TXCIE))))	return;	//처리중/송신중 수신 무시

	if(st & ((1<<FE)|(1<<DOR)))	bRxErr = 1;

	if(bRxLen < MB_RX_SIZE)	bRxBuf[bRxLen++] = data;
	else					bRxErr = 1;

	bRxIdleCnt = 0;
	bRxBusy = 1;
}

ISR(USART_UDRE_vect)
{
	if(bTxIdx < bTxLen)
	{
		UDR = bTxBuf[bTxIdx++];
	}
	else
	{
		UCSRB &= ~(1<<UDRIE);
		UCSRA |= (1<<TXC);					//TXC 플래그 클리어 (1 쓰기)
		UCSRB |= (1<<TXCIE);				//마지막 바이트 송신 완료 대기
	}
}

ISR(USART_TXC_vect)
{
	UCSRB &= ~(1<<TXCIE);
	RS485_RX_MODE;							//송신 완료 -> 수신으로
}

void Port_Init(void)
{
	//port init //1=OUT , 0=IN
	DDRB = 0x00;	//PB0~2 DIP(외부 풀업), PB3~5 ISP, PB6~7 XTAL
	DDRC = 0x20;	//00100000 PC0~4 ADC , PC5 STATE LED
	DDRD = 0xfe;	//11111110 PD0 RxD , PD1 TxD , PD2 RS485_Drive , PD3~7 스테이션 LED

	PORTB = 0x00;
	PORTC = 0x20;	//ADC 입력 풀업 없음 , STATE LED OFF
	PORTD = 0xfb;	//11111011 LED OFF(HIGH) , RS485 수신 , TxD idle HIGH
}

void Timer1_Init(void)
{
	TCCR1A = 0;
	TCCR1B = (1<<WGM12)|(1<<CS11)|(1<<CS10);	//CTC , 64분주
	TCNT1H = 0x00;
	TCNT1L = 0x00;
	OCR1A = 249;								//16MHz/64/(1+249) = 1000Hz
	TIMSK |= (1<<OCIE1A);
}

void ADC_Init(void)
{
	//AREF 핀에 DC5V 가 직접 연결되어 있으므로 REFS1:0 = 00 (외부 AREF) 사용
	ADMUX = 0x00;
	ADCSRA = (1<<ADEN)|(1<<ADIE)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);	//16MHz/128 = 125kHz , 1회 104us
}

void UART_init(void)
{
	UCSRA = (1<<U2X);
	UBRRH = (uint8_t)(MB_UBRR >> 8);
	UBRRL = (uint8_t)(MB_UBRR);
	UCSRC = (1<<URSEL)|(1<<UCSZ1)|(1<<UCSZ0);		//8N1
	UCSRB = (1<<RXCIE)|(1<<RXEN)|(1<<TXEN);
}

int main(void)
{
	uint8_t k;

	Port_Init();
	Timer1_Init();
	ADC_Init();
	UART_init();

	//스테이션 LED 점검 1ST -> 5ST
	LED_ON;
	for(k=0;k<CH_NUM;k++)
	{
		fcStLed(k,1);
		_delay_ms(150);
		fcStLed(k,0);
	}

	fcReadData();
	fcIdCheck();

	MDregister[M_FW_VERSION] = FW_VERSION;

	wdt_enable(WDTO_1S);
	sei();

	while(1)
	{
		wdt_reset();

		if(bRxDone)
		{
			fcModbusProcess();
		}

		if(b10msFg)
		{
			b10msFg = 0;

			if(bCmdFg)	fcCmdProcess();
		}

		if(bAdcFg)	//100ms
		{
			bAdcFg = 0;
			fcLoadCell();
		}

		if(b100msFg)
		{
			b100msFg = 0;

			//전원 투입 영점
			if(bPowerOnFg==0)
			{
				if(++bPowerOnCnt >= POWERON_ZERO_TIME)
				{
					bPowerOnFg = 1;
#if ZERO_AT_POWERON
					fcPowerOnZero();
#endif
				}
			}

			//통신 끊김 체크
			if(bCommCnt < COMM_TIMEOUT)	bCommCnt++;
			bCommOk = (bCommCnt < COMM_TIMEOUT);

			//500ms 점멸 플래그 (센서 이상 채널 LED 점멸)
			if(++bBlinkCnt >= 5)
			{
				bBlinkCnt = 0;
				bBlinkFg ^= 1;
			}

			fcStLedControl();
			fcStateLed();

			if(++b1SecCnt >= 10)	//1sec
			{
				b1SecCnt = 0;
				fcIdCheck();
			}
		}

		if(bSaveFg)
		{
			fcSaveData();
		}
	}
}

//스테이션 LED 1ST~5ST
void fcStLed(uint8_t num, uint8_t on)
{
	if(on)
	{
		if(num==0)		ST1_LED_ON;
		else if(num==1)	ST2_LED_ON;
		else if(num==2)	ST3_LED_ON;
		else if(num==3)	ST4_LED_ON;
		else if(num==4)	ST5_LED_ON;
	}
	else
	{
		if(num==0)		ST1_LED_OFF;
		else if(num==1)	ST2_LED_OFF;
		else if(num==2)	ST3_LED_OFF;
		else if(num==3)	ST4_LED_OFF;
		else if(num==4)	ST5_LED_OFF;
	}
}

void fcStLedControl(void)
{
	uint8_t k,bit;

	for(k=0;k<CH_NUM;k++)
	{
		bit = 1<<k;

		if(bLedMode == LED_MODE_MANUAL)		//메인보드 제어
		{
			fcStLed(k, (bLedManual & bit) ? 1 : 0);
		}
		else								//AUTO : 감지 ON , 센서 이상은 점멸
		{
			if(bSensorErr & bit)	fcStLed(k, bBlinkFg);
			else					fcStLed(k, (bDetect & bit) ? 1 : 0);
		}
	}
}

//STATE LED (D1)
// 전원투입 영점 전 : 켜짐
// 센서 이상       : 100ms 점멸
// 통신 정상       : 500ms 점멸
// 통신 없음       : 1sec 점멸
void fcStateLed(void)
{
	uint8_t time;

	if(bPowerOnFg==0)
	{
		LED_ON;
		return;
	}

	if(bSensorErr)		time = 1;
	else if(bCommOk)	time = 5;
	else				time = 10;

	if(++bLedCnt >= time)
	{
		bLedCnt = 0;

		bLedFg = ~bLedFg;
		if(bLedFg)	LED_ON;
		else		LED_OFF;
	}
}

//DIP S/W -> 보드 ID , 슬레이브 주소
void fcIdCheck(void)
{
	bBoardId = IN_DIP_ID;
	bSlaveAddr = bBoardId + MB_ADDR_BASE;

	MDregister[M_BOARD_ID] = bBoardId;
	MDregister[M_LC_START_NO] = bBoardId*CH_NUM + 1;	//ID0 -> Load Cell 1~5 , ID1 -> 6~10
}

//100ms 마다 : 이동평균 , 무게 계산 , 감지 판단
void fcLoadCell(void)
{
	uint8_t ch,k,bit;
	uint16_t raw[CH_NUM];
	uint32_t sum;
	int32_t diff,w;
	uint16_t off;

	cli();
	for(ch=0;ch<CH_NUM;ch++)	raw[ch] = lADC_Buf[ch];
	sei();

	for(ch=0;ch<CH_NUM;ch++)
	{
		bit = 1<<ch;

		//이동평균 (처음에는 버퍼 전체를 첫 값으로 채움)
		if(bFilterFull==0)
		{
			for(k=0;k<FILTER_NUM;k++)	wRawBuf[ch][k] = raw[ch];
		}
		else
		{
			wRawBuf[ch][bFilterIdx] = raw[ch];
		}

		sum = 0;
		for(k=0;k<FILTER_NUM;k++)	sum += wRawBuf[ch][k];
		wRaw[ch] = sum / FILTER_NUM;

		//센서 이상 : ADC 범위를 벗어난 상태가 1초 지속 (단선/포화)
		if((raw[ch] <= RAW_ERR_LOW) || (raw[ch] >= RAW_ERR_HIGH))
		{
			if((bSensorErr & bit)==0)
			{
				if(++bErrCnt[ch] >= SENSOR_ERR_TIME)
				{
					bErrCnt[ch] = 0;
					bSensorErr |= bit;
				}
			}
			else	bErrCnt[ch] = 0;
		}
		else
		{
			if(bSensorErr & bit)
			{
				if(++bErrCnt[ch] >= SENSOR_ERR_TIME)
				{
					bErrCnt[ch] = 0;
					bSensorErr &= ~bit;
				}
			}
			else	bErrCnt[ch] = 0;
		}

		//무게 = (raw - 영점) x 100.0g / 스팬
		diff = (int32_t)wRaw[ch] - (int32_t)wZero[ch];
		if(lSpan[ch])	w = (diff * SPAN_REF_WEIGHT) / lSpan[ch];
		else			w = 0;

		if(w > WEIGHT_LIMIT)	w = WEIGHT_LIMIT;
		if(w < -WEIGHT_LIMIT)	w = -WEIGHT_LIMIT;
		iWeight[ch] = (int16_t)w;

		//과부하
		if(iWeight[ch] >= OVER_WEIGHT)	bOverFg |= bit;
		else							bOverFg &= ~bit;

		//감지 (히스테리시스)
		off = (wDetectOff < wDetectOn) ? wDetectOff : wDetectOn;
		if(bSensorErr & bit)					bDetect &= ~bit;
		else if(iWeight[ch] >= (int16_t)wDetectOn)	bDetect |= bit;
		else if(iWeight[ch] <  (int16_t)off)		bDetect &= ~bit;

		//송신 레지스터
		MDregister[M_WEIGHT1+ch] = (uint16_t)iWeight[ch];
		MDregister[M_RAW1+ch]    = wRaw[ch];
		MDregister[M_ZERO1+ch]   = wZero[ch];
		if(lSpan[ch] > 32767)			MDregister[M_SPAN1+ch] = 32767;
		else if(lSpan[ch] < -32767)		MDregister[M_SPAN1+ch] = (uint16_t)(-32767);
		else							MDregister[M_SPAN1+ch] = (uint16_t)lSpan[ch];
	}

	if(++bFilterIdx >= FILTER_NUM)	bFilterIdx = 0;
	bFilterFull = 1;

	MDregister[M_DETECT] = bDetect;
	MDregister[M_ERROR]  = bSensorErr | ((uint16_t)bOverFg << 8);
}

//전원 투입 영점 : 저장 영점 기준 ±POWERON_ZERO_RANGE 이내 채널만 (RAM 적용, EEPROM 저장 안함)
void fcPowerOnZero(void)
{
	uint8_t ch;

	for(ch=0;ch<CH_NUM;ch++)
	{
		if(bSensorErr & (1<<ch))	continue;

		//영점 미설정(0) 이거나 저장 영점 기준 범위 이내
		if((wZero[ch]==0) || ((iWeight[ch] <= POWERON_ZERO_RANGE) && (iWeight[ch] >= -POWERON_ZERO_RANGE)))
		{
			wZero[ch] = wRaw[ch];
		}
	}
}

//메인보드 명령 처리 (영점 / 스팬 / 초기화)
void fcCmdProcess(void)
{
	uint8_t ch,cmd,res=CMD_RES_OK;
	int32_t diff;

	bCmdFg = 0;
	cmd = bCmd;

	if(cmd==CMD_NONE)	return;

	if((cmd>=CMD_ZERO1 && cmd<(CMD_ZERO1+CH_NUM)) || (cmd==CMD_ZERO_ALL))	//영점
	{
		for(ch=0;ch<CH_NUM;ch++)
		{
			if((cmd!=CMD_ZERO_ALL) && (ch!=(cmd-CMD_ZERO1)))	continue;

			if(bSensorErr & (1<<ch))	res = CMD_RES_FAIL;
			else						wZero[ch] = wRaw[ch];
		}
		bSaveFg |= SAVE_ZERO;
	}
	else if((cmd>=CMD_SPAN1 && cmd<(CMD_SPAN1+CH_NUM)) || (cmd==CMD_SPAN_ALL))	//스팬 (분동 올린 상태)
	{
		for(ch=0;ch<CH_NUM;ch++)
		{
			if((cmd!=CMD_SPAN_ALL) && (ch!=(cmd-CMD_SPAN1)))	continue;

			diff = (int32_t)wRaw[ch] - (int32_t)wZero[ch];

			if((bSensorErr & (1<<ch)) || ((diff < SPAN_MIN_RAW) && (diff > -SPAN_MIN_RAW)))
			{
				res = CMD_RES_FAIL;
			}
			else
			{
				lSpan[ch] = (diff * SPAN_REF_WEIGHT) / wCalWeight;	//100.0g 기준으로 환산 (역방향 결선은 음수)
			}
		}
		bSaveFg |= SAVE_SPAN;
	}
	else if(cmd==CMD_CAL_INIT)
	{
		fcCalInit();
		bSaveFg |= (SAVE_ZERO|SAVE_SPAN|SAVE_SET);
	}
	else
	{
		res = CMD_RES_UNKNOWN;
	}

	bCmd = CMD_NONE;
	MDregister[M_CMD] = CMD_NONE;
	MDregister[M_CMD_RESULT] = res;
}

//================================================================
// 통신  메인보드(마스터) <-> 컨트롤보드(슬레이브 DIP+1) : Modbus RTU 9600 8N1
//  FC03/FC04 읽기 : 0~39
//  FC06/FC16 쓰기 : 20~25 , 주소 0(브로드캐스트) 쓰기는 응답 없음
//================================================================
uint16_t fcMbCrc(volatile uint8_t *buf, uint8_t len)
{
	uint16_t crc = 0xffff;
	uint8_t i,j;

	for(i=0;i<len;i++){
		crc ^= buf[i];
		for(j=0;j<8;j++){
			if(crc & 0x0001)	crc = (crc >> 1) ^ 0xa001;
			else				crc >>= 1;
		}
	}
	return crc;
}

void fcMbSend(uint8_t len)
{
	uint16_t crc;

	crc = fcMbCrc(bTxBuf, len);
	bTxBuf[len++] = crc & 0xff;		//CRC Low 먼저
	bTxBuf[len++] = crc >> 8;

	bTxLen = len;
	bTxDelayCnt = 0;
	bTxReq = 1;						//MB_TX_DELAY 후 타이머에서 송신 시작
}

void fcMbException(uint8_t func, uint8_t code)
{
	bTxBuf[0] = bSlaveAddr;
	bTxBuf[1] = func | 0x80;
	bTxBuf[2] = code;
	fcMbSend(3);
}

//쓰기 값 범위 검사
uint8_t fcRegCheck(uint16_t addr, uint16_t val)
{
	if(addr < M_W_START || addr > M_W_END)	return MB_EX_ADDR;

	switch(addr){
		case M_LED_MODE :
			if(val > LED_MODE_MANUAL)	return MB_EX_VALUE;
		break;
		case M_LED_MANUAL :
			if(val > 0x1f)	return MB_EX_VALUE;
		break;
		case M_DETECT_ON :
		case M_DETECT_OFF :
			if(val < 1 || val > DETECT_MAX)	return MB_EX_VALUE;
		break;
		case M_CAL_WEIGHT :
			if(val < CAL_WEIGHT_MIN || val > CAL_WEIGHT_MAX)	return MB_EX_VALUE;
		break;
		case M_CMD :
			if(val > 0xff)	return MB_EX_VALUE;
		break;
		default:
		break;
	}
	return 0;
}

void fcRegWrite(uint16_t addr, uint16_t val)
{
	MDregister[addr] = val;

	switch(addr){
		case M_LED_MODE :
			bLedMode = val;
			bSaveFg |= SAVE_SET;
		break;
		case M_LED_MANUAL :
			bLedManual = val;			//저장 안함
		break;
		case M_DETECT_ON :
			wDetectOn = val;
			bSaveFg |= SAVE_SET;
		break;
		case M_DETECT_OFF :
			wDetectOff = val;
			bSaveFg |= SAVE_SET;
		break;
		case M_CAL_WEIGHT :
			wCalWeight = val;
			bSaveFg |= SAVE_SET;
		break;
		case M_CMD :
			bCmd = val;
			bCmdFg = 1;					//같은 값이라도 쓰기 때마다 처리
		break;
		default:
		break;
	}
}

void fcModbusProcess(void)
{
	volatile uint8_t *rx = bRxBuf;
	uint8_t n,i,func,bcast,txlen,ex;
	uint16_t addr,qty,val,crc;

	n = bRxLen;

	if(bRxErr || n < 8)	goto rx_end;

	crc = fcMbCrc(rx, n-2);
	if(((crc & 0xff) != rx[n-2]) || ((crc >> 8) != rx[n-1]))	goto rx_end;	//CRC 에러 -> 무응답

	if((rx[0] != bSlaveAddr) && (rx[0] != 0))	goto rx_end;	//다른 보드 주소
	bcast = (rx[0] == 0);

	bCommCnt = 0;				//통신 정상

	func = rx[1];
	addr = ((uint16_t)rx[2] << 8) | rx[3];
	qty  = ((uint16_t)rx[4] << 8) | rx[5];

	if(func == 0x03 || func == 0x04)	//Read Holding / Input Registers
	{
		if(bcast)	goto rx_end;

		if(qty == 0 || qty > MB_READ_MAX)	fcMbException(func, MB_EX_VALUE);
		else if((addr + qty) > MB_REG_NUM)	fcMbException(func, MB_EX_ADDR);
		else
		{
			bTxBuf[0] = bSlaveAddr;
			bTxBuf[1] = func;
			bTxBuf[2] = qty * 2;
			txlen = 3;

			for(i=0;i<qty;i++){
				val = MDregister[addr + i];
				bTxBuf[txlen++] = val >> 8;
				bTxBuf[txlen++] = val & 0xff;
			}
			fcMbSend(txlen);
		}
	}
	else if(func == 0x06)	//Write Single Register
	{
		ex = fcRegCheck(addr, qty);		//qty 위치 = 값

		if(ex)
		{
			if(!bcast)	fcMbException(func, ex);
		}
		else
		{
			fcRegWrite(addr, qty);

			if(!bcast)
			{
				for(i=0;i<6;i++)	bTxBuf[i] = rx[i];	//요청 그대로 응답
				fcMbSend(6);
			}
		}
	}
	else if(func == 0x10)	//Write Multiple Registers
	{
		ex = 0;

		if(n < 11 || qty == 0 || rx[6] != qty*2 || n != (9 + qty*2))	ex = MB_EX_VALUE;
		else
		{
			for(i=0;i<qty;i++){		//전체 검사 후 쓰기
				val = ((uint16_t)rx[7 + i*2] << 8) | rx[8 + i*2];
				ex = fcRegCheck(addr + i, val);
				if(ex)	break;
			}
		}

		if(ex)
		{
			if(!bcast)	fcMbException(func, ex);
		}
		else
		{
			for(i=0;i<qty;i++){
				val = ((uint16_t)rx[7 + i*2] << 8) | rx[8 + i*2];
				fcRegWrite(addr + i, val);
			}

			if(!bcast)
			{
				for(i=0;i<6;i++)	bTxBuf[i] = rx[i];
				fcMbSend(6);
			}
		}
	}
	else
	{
		if(!bcast)	fcMbException(func, MB_EX_FUNC);
	}

rx_end:
	cli();
	bRxLen = 0;
	bRxErr = 0;
	bRxDone = 0;
	sei();
}

//================================================================
// EEPROM
//================================================================
void fcEepWriteWord(uint16_t addr, uint16_t data)
{
	EEPROM_WriteByte(addr,   (data >> 8) & 0xff);
	EEPROM_WriteByte(addr+1, data & 0xff);
}

uint16_t fcEepReadWord(uint16_t addr)
{
	uint16_t data;

	data = EEPROM_ReadByte(addr);
	data <<= 8;
	data |= EEPROM_ReadByte(addr+1);
	return data;
}

void fcCalInit(void)
{
	uint8_t k;

	for(k=0;k<CH_NUM;k++)
	{
		wZero[k] = (bFilterFull && !(bSensorErr & (1<<k))) ? wRaw[k] : 0;	//현재값을 영점으로 (전원 투입 직후는 0 = 미설정)
		lSpan[k] = DEF_SPAN;
	}
	wDetectOn  = DEF_DETECT_ON;
	wDetectOff = DEF_DETECT_OFF;
	wCalWeight = DEF_CAL_WEIGHT;
	bLedMode   = LED_MODE_AUTO;

	MDregister[M_LED_MODE]   = bLedMode;
	MDregister[M_DETECT_ON]  = wDetectOn;
	MDregister[M_DETECT_OFF] = wDetectOff;
	MDregister[M_CAL_WEIGHT] = wCalWeight;
}

//한 번에 하나씩 저장 (main loop 지연 최소화)
void fcSaveData(void)
{
	uint8_t k;
	uint32_t span;

	if(bSaveFg & SAVE_ZERO){
		for(k=0;k<CH_NUM;k++)
		{
			fcEepWriteWord(ADDR_ZERO + k*2, wZero[k]);
		}
		bSaveFg &= ~SAVE_ZERO;
	}
	else if(bSaveFg & SAVE_SPAN){
		for(k=0;k<CH_NUM;k++)
		{
			span = (uint32_t)lSpan[k];
			fcEepWriteWord(ADDR_SPAN + k*4,     (uint16_t)(span >> 16));
			fcEepWriteWord(ADDR_SPAN + k*4 + 2, (uint16_t)(span & 0xffff));
		}
		bSaveFg &= ~SAVE_SPAN;
	}
	else if(bSaveFg & SAVE_SET){
		fcEepWriteWord(ADDR_DETECT_ON,  wDetectOn);
		fcEepWriteWord(ADDR_DETECT_OFF, wDetectOff);
		fcEepWriteWord(ADDR_CAL_WEIGHT, wCalWeight);
		EEPROM_WriteByte(ADDR_LED_MODE, bLedMode);
		bSaveFg &= ~SAVE_SET;
	}
}

void fcReadData(void)
{
	uint8_t k;
	uint16_t addr_temp;
	uint32_t span;

	addr_temp = fcEepReadWord(ADDR_INIT_H);

	if(addr_temp != INIT_DATA)		//처음 전원 투입 -> 초기값 저장
	{
		fcCalInit();

		bSaveFg = (SAVE_ZERO|SAVE_SPAN|SAVE_SET);
		while(bSaveFg)	fcSaveData();

		fcEepWriteWord(ADDR_INIT_H, INIT_DATA);
	}

	for(k=0;k<CH_NUM;k++)
	{
		wZero[k] = fcEepReadWord(ADDR_ZERO + k*2);

		span = fcEepReadWord(ADDR_SPAN + k*4);
		span <<= 16;
		span |= fcEepReadWord(ADDR_SPAN + k*4 + 2);
		lSpan[k] = (int32_t)span;
		if(lSpan[k]==0)	lSpan[k] = DEF_SPAN;
	}

	wDetectOn  = fcEepReadWord(ADDR_DETECT_ON);
	wDetectOff = fcEepReadWord(ADDR_DETECT_OFF);
	wCalWeight = fcEepReadWord(ADDR_CAL_WEIGHT);
	bLedMode   = EEPROM_ReadByte(ADDR_LED_MODE);

	//범위 밖이면 초기값
	if(wDetectOn < 1 || wDetectOn > DETECT_MAX)		wDetectOn = DEF_DETECT_ON;
	if(wDetectOff < 1 || wDetectOff > DETECT_MAX)	wDetectOff = DEF_DETECT_OFF;
	if(wCalWeight < CAL_WEIGHT_MIN || wCalWeight > CAL_WEIGHT_MAX)	wCalWeight = DEF_CAL_WEIGHT;
	if(bLedMode > LED_MODE_MANUAL)	bLedMode = LED_MODE_AUTO;

	MDregister[M_LED_MODE]   = bLedMode;
	MDregister[M_LED_MANUAL] = bLedManual;
	MDregister[M_DETECT_ON]  = wDetectOn;
	MDregister[M_DETECT_OFF] = wDetectOff;
	MDregister[M_CAL_WEIGHT] = wCalWeight;
}

void EEPROM_WriteByte(uint16_t Address, uint8_t Data)
{
	uint8_t sreg;

	if(EEPROM_ReadByte(Address) == Data)	return;	//같은 값이면 쓰지 않음 (수명)

	while(EECR & (1<<EEWE));	// write가 완료될때까지 기다림
	wdt_reset();

	EEAR = Address;	//주소 지정
	EEDR = Data;	//데이터 지정

	sreg = SREG;
	cli();						//EEMWE -> EEWE 4클럭 이내 (인터럽트 금지)
	EECR |= (1<<EEMWE);			//마스터 쓰기 비트에 1 인가
	EECR |= (1<<EEWE);			//쓰기 비트에 1 인가
	SREG = sreg;
}

uint8_t EEPROM_ReadByte(uint16_t Address)
{
	while(EECR & (1<<EEWE));	// write가 완료될때까지 기다림

	EEAR = Address;				//주소 지정
	EECR |= (1<<EERE);			// 읽기 비트에 1 인가

	return EEDR;				// 읽은 데이터 값 Return
}
