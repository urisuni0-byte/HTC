/*
 * MAIN.c
 *
 * 메인보드 (ATmega2560 16MHz)
 *  - RS485 (USART3 RXD3/TXD3) : 로드셀 컨트롤 보드(SVROCB) 2장을 Modbus RTU 마스터로 폴링
 *            (보드1 = Load Cell 1~5 , 보드2 = Load Cell 6~10)
 *  - RS232 (USART0 RXD0/TXD0) : 무게(0.1g)와 유무 상태를 상위 장치로 송신 , 상위 장치 명령(영점/스팬/LED/감지값) 중계
 */
//261112
//최초 작성
//261117  v1.01
//로드셀 보드 영점 에러(레지스터 15) 읽기 , 상태 'Z'
//RS485 포트 USART1 -> USART3 (RXD3/TXD3) 변경 , RS232 는 USART0 (RXD0/TXD0)
//RS485 요청 송신 완료 전 수신 바이트 무시 (에코/잡음으로 응답 깨짐 방지) , $DBG 통신 진단 명령 추가

#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <string.h>
#include "define.h"

typedef struct{
	uint8_t  board;		//0 ~ BOARD_NUM-1
	uint8_t  type;		//JOB_WRITE , JOB_CMD
	uint8_t  reg;		//시작 레지스터
	uint8_t  qty;		//1~2
	uint16_t val[2];
} job_t;

void Port_Init(void);
void Timer1_Init(void);
void UART_init(void);
void fcMbMaster(void);
void fcPcProcess(void);
void fcSendData(void);
void fcStatusMake(char *st);
void fcAutoSend(void);
void fcStateLed(void);
void fcPcPutc(char c);
void fcOutStart(void);
void fcOutStr(const char *s);
void fcOutUint(uint16_t v);
void fcOutEnd(void);
void fcReadData(void);
void EEPROM_WriteByte(uint16_t Address, uint8_t Data);
uint8_t EEPROM_ReadByte(uint16_t Address);

//timer
volatile uint8_t b10msCnt=0,b10msFg=0;
volatile uint8_t b100msCnt=0,b100msFg=0;
volatile uint16_t wMbWaitCnt=0;		//응답 대기 ms
volatile uint16_t wMbGapCnt=0;		//프레임 간격 ms
volatile uint16_t wHoldCnt[BOARD_NUM]={0,};	//보드별 대기 ms
volatile uint16_t wAutoCnt=0;
uint8_t bLedCnt=0,bLedFg=0;

//load cell data
int16_t  iWeight[LC_NUM]={0,};		//0.1g
uint8_t  bDetect[BOARD_NUM]={0,};		//bit0~4
uint16_t wError[BOARD_NUM]={0,};		//bit0~4 센서 이상 , bit8~12 과부하
uint8_t  bZeroErr[BOARD_NUM]={0,};	//bit0~4 영점 에러
uint8_t  bOnline[BOARD_NUM]={0,};
uint8_t  bFailCnt[BOARD_NUM]={0,};
uint16_t wBoardFw[BOARD_NUM]={0,};

//통신 진단 ($DBG)
uint16_t wDbgOk[BOARD_NUM]={0,};		//정상 응답
uint16_t wDbgTimeout[BOARD_NUM]={0,};	//무응답
uint16_t wDbgBad[BOARD_NUM]={0,};		//응답은 왔으나 깨짐 (CRC/길이/주소)
uint8_t  bDbgLastLen[BOARD_NUM]={0,};	//마지막 수신 바이트 수
char     cStatusOld[LC_NUM+1]={0,};

//modbus master
volatile uint8_t bMbRxBuf[MB_RX_SIZE];
volatile uint8_t bMbRxLen=0,bMbRxErr=0;
volatile uint8_t bMbRxIdleCnt=0,bMbRxBusy=0,bMbRxDone=0;
volatile uint8_t bMbTxBuf[MB_TX_SIZE];
volatile uint8_t bMbTxLen=0,bMbTxIdx=0,bMbTxEnd=0;
volatile uint8_t bMbState=MS_IDLE;
uint8_t  bReqAddr=0,bReqFunc=0,bReqBoard=0;
uint16_t wReqReg=0,wReqQty=0;
uint8_t  bReqType=0;				//0 = 폴링 , 1 = 작업 쓰기 , 2 = 작업 결과 읽기
uint8_t  bPollBoard=0;

//job (상위 장치 명령 -> 보드 쓰기)
job_t    stJob[JOB_MAX];
uint8_t  bJobNum=0,bJobIdx=0,bJobPhase=0,bJobRetry=0;
uint8_t  bJobResult=RES_OK;
char     cJobName[8]={0,};			//ACK 에 쓸 명령 이름

//pc rs232
volatile char    cPcRxBuf[PC_RX_SIZE];
volatile uint8_t bPcRxLen=0,bPcLineFg=0;
volatile char    cPcTxBuf[PC_TX_SIZE];
volatile uint16_t wPcTxHead=0,wPcTxTail=0;
char     cPcLine[PC_RX_SIZE];
uint8_t  bOutCs=0;
uint16_t wAutoTime=DEF_AUTO_TIME;


ISR(TIMER1_COMPA_vect)	//1ms
{
	uint8_t k;

	if(++b10msCnt >= 10)//10ms timer
	{
		b10msCnt = 0;
		b10msFg = 1;

		if(++b100msCnt >= 10){
			b100msCnt = 0;
			b100msFg = 1;
		}
	}

	//RS485 응답 프레임 끝 검출
	if(bMbRxBusy)
	{
		if(++bMbRxIdleCnt >= MB_FRAME_GAP)
		{
			bMbRxBusy = 0;
			bMbRxDone = 1;
		}
	}

	if(bMbTxEnd && wMbWaitCnt < 0xffff)	wMbWaitCnt++;	//송신 완료 후부터 응답 대기 시간
	if(wMbGapCnt < 0xffff)	wMbGapCnt++;

	for(k=0;k<BOARD_NUM;k++)
	{
		if(wHoldCnt[k])	wHoldCnt[k]--;
	}

	if(wAutoCnt < 0xffff)	wAutoCnt++;
}

//================================================================
// USART3 : RS485 (PJ0 RXD3 / PJ1 TXD3)
//================================================================
ISR(USART3_RX_vect)
{
	uint8_t st,data;

	st = UCSR3A;
	data = UDR3;

	//응답 대기중 + 요청 송신 완료 후에만 받음 (송신중 자기 에코 / 잡음 무시)
	if(bMbState != MS_WAIT || bMbRxDone || bMbTxEnd==0)	return;

	if(st & ((1<<FE3)|(1<<DOR3)))	bMbRxErr = 1;

	if(bMbRxLen < MB_RX_SIZE)	bMbRxBuf[bMbRxLen++] = data;
	else						bMbRxErr = 1;

	bMbRxIdleCnt = 0;
	bMbRxBusy = 1;
}

ISR(USART3_UDRE_vect)
{
	if(bMbTxIdx < bMbTxLen)
	{
		UDR3 = bMbTxBuf[bMbTxIdx++];
	}
	else
	{
		UCSR3B &= ~(1<<UDRIE3);
		UCSR3A |= (1<<TXC3);				//TXC 플래그 클리어
		UCSR3B |= (1<<TXCIE3);				//마지막 바이트 송신 완료 대기
	}
}

ISR(USART3_TX_vect)
{
	UCSR3B &= ~(1<<TXCIE3);
	RS485_RX_MODE;							//송신 완료 -> 수신
	wMbWaitCnt = 0;
	bMbTxEnd = 1;							//응답 대기 시작
}

//================================================================
// USART0 : RS232 상위 장치
//================================================================
ISR(USART0_RX_vect)
{
	char c;

	c = UDR0;

	if(bPcLineFg)	return;					//이전 명령 처리중

	if(c=='\r' || c=='\n')
	{
		if(bPcRxLen)
		{
			cPcRxBuf[bPcRxLen] = 0;
			bPcLineFg = 1;
		}
	}
	else if(c=='$')							//프레임 시작 -> 처음부터
	{
		bPcRxLen = 0;
		cPcRxBuf[bPcRxLen++] = c;
	}
	else if(bPcRxLen && bPcRxLen < (PC_RX_SIZE-1))
	{
		cPcRxBuf[bPcRxLen++] = c;
	}
}

ISR(USART0_UDRE_vect)
{
	if(wPcTxTail != wPcTxHead)
	{
		UDR0 = cPcTxBuf[wPcTxTail];
		if(++wPcTxTail >= PC_TX_SIZE)	wPcTxTail = 0;
	}
	else
	{
		UCSR0B &= ~(1<<UDRIE0);
	}
}

void Port_Init(void)
{
	//port init //1=OUT , 0=IN
	DDRA = 0x00;
	DDRB = 0x00;
	DDRD = 0x00;
	DDRE = 0x00;

	RS485_DDR_INIT;
	RS485_RX_MODE;
	LED_DDR_INIT;
	LED_OFF;

	PORTJ |= (1<<PJ0);	//RXD3 풀업
	PORTE |= (1<<PE0);	//RXD0 풀업
}

void Timer1_Init(void)
{
	TCCR1A = 0;
	TCCR1B = (1<<WGM12)|(1<<CS11)|(1<<CS10);	//CTC , 64분주
	TCNT1 = 0;
	OCR1A = 249;								//16MHz/64/(1+249) = 1000Hz
	TIMSK1 |= (1<<OCIE1A);
}

void UART_init(void)
{
	//USART0 : RS232 115200 8N1
	UCSR0A = (1<<U2X0);
	UBRR0H = (uint8_t)(PC_UBRR >> 8);
	UBRR0L = (uint8_t)(PC_UBRR);
	UCSR0C = (1<<UCSZ01)|(1<<UCSZ00);
	UCSR0B = (1<<RXCIE0)|(1<<RXEN0)|(1<<TXEN0);

	//USART3 : RS485 9600 8N1
	UCSR3A = (1<<U2X3);
	UBRR3H = (uint8_t)(MB_UBRR >> 8);
	UBRR3L = (uint8_t)(MB_UBRR);
	UCSR3C = (1<<UCSZ31)|(1<<UCSZ30);
	UCSR3B = (1<<RXCIE3)|(1<<RXEN3)|(1<<TXEN3);
}

int main(void)
{
	MCUSR = 0;
	wdt_disable();

	Port_Init();
	Timer1_Init();
	UART_init();

	fcReadData();

	memset(cStatusOld, ST_COMM_ERR, LC_NUM);

	LED_ON;
	_delay_ms(1000);		//로드셀 보드 부팅 대기 (LED 점검 약 0.8초)
	LED_OFF;

	wdt_enable(WDTO_1S);
	sei();

	//시작 알림 $BOOT,버전
	fcOutStart();
	fcOutStr("BOOT,");
	fcOutUint(FW_VERSION);
	fcOutEnd();

	while(1)
	{
		wdt_reset();

		fcMbMaster();

		if(bPcLineFg)
		{
			fcPcProcess();
		}

		if(b10msFg)
		{
			b10msFg = 0;
			fcAutoSend();
		}

		if(b100msFg)
		{
			b100msFg = 0;
			fcStateLed();
		}
	}
}

//STATE LED : 모든 보드 통신 정상 500ms 점멸 , 끊긴 보드 있으면 100ms 점멸
void fcStateLed(void)
{
	uint8_t k,time=5;

	for(k=0;k<BOARD_NUM;k++)
	{
		if(bOnline[k]==0)	time = 1;
	}

	if(++bLedCnt >= time)
	{
		bLedCnt = 0;

		bLedFg = ~bLedFg;
		if(bLedFg)	LED_ON;
		else		LED_OFF;
	}
}

//================================================================
// RS485 Modbus RTU 마스터
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

	crc = fcMbCrc(bMbTxBuf, len);
	bMbTxBuf[len++] = crc & 0xff;		//CRC Low 먼저
	bMbTxBuf[len++] = crc >> 8;

	cli();
	bMbRxLen = 0;
	bMbRxErr = 0;
	bMbRxBusy = 0;
	bMbRxDone = 0;
	bMbTxEnd = 0;
	wMbWaitCnt = 0;
	bMbState = MS_WAIT;
	sei();

	bMbTxLen = len;
	bMbTxIdx = 0;
	RS485_TX_MODE;
	UCSR3B |= (1<<UDRIE3);				//인터럽트 송신 시작
}

//FC03 읽기 요청
void fcMbReadReq(uint8_t board, uint16_t reg, uint16_t qty)
{
	bReqBoard = board;
	bReqAddr  = board + BOARD_ADDR_BASE;
	bReqFunc  = 0x03;
	wReqReg   = reg;
	wReqQty   = qty;

	bMbTxBuf[0] = bReqAddr;
	bMbTxBuf[1] = 0x03;
	bMbTxBuf[2] = reg >> 8;
	bMbTxBuf[3] = reg & 0xff;
	bMbTxBuf[4] = qty >> 8;
	bMbTxBuf[5] = qty & 0xff;
	fcMbSend(6);
}

//FC16 쓰기 요청
void fcMbWriteReq(uint8_t board, uint16_t reg, uint8_t qty, uint16_t *val)
{
	uint8_t i,len;

	bReqBoard = board;
	bReqAddr  = board + BOARD_ADDR_BASE;
	bReqFunc  = 0x10;
	wReqReg   = reg;
	wReqQty   = qty;

	bMbTxBuf[0] = bReqAddr;
	bMbTxBuf[1] = 0x10;
	bMbTxBuf[2] = reg >> 8;
	bMbTxBuf[3] = reg & 0xff;
	bMbTxBuf[4] = 0;
	bMbTxBuf[5] = qty;
	bMbTxBuf[6] = qty * 2;
	len = 7;
	for(i=0;i<qty;i++)
	{
		bMbTxBuf[len++] = val[i] >> 8;
		bMbTxBuf[len++] = val[i] & 0xff;
	}
	fcMbSend(len);
}

//응답 검사 : 0 = 정상 , RES_COMM = 무응답/깨짐 , RES_REJECT = 예외 응답
uint8_t fcMbCheck(void)
{
	volatile uint8_t *rx = bMbRxBuf;
	uint8_t n;
	uint16_t crc;

	n = bMbRxLen;

	if(bMbRxErr || n < 5)	return RES_COMM;

	crc = fcMbCrc(rx, n-2);
	if(((crc & 0xff) != rx[n-2]) || ((crc >> 8) != rx[n-1]))	return RES_COMM;
	if(rx[0] != bReqAddr)	return RES_COMM;

	if(rx[1] == (bReqFunc | 0x80))	return RES_REJECT;	//예외 응답 (보드는 살아 있음)
	if(rx[1] != bReqFunc)	return RES_COMM;

	if(bReqFunc == 0x03)
	{
		if(rx[2] != wReqQty*2 || n != (5 + wReqQty*2))	return RES_COMM;
	}
	else //0x10
	{
		if(n != 8)	return RES_COMM;
	}
	return RES_OK;
}

uint16_t fcMbGetReg(uint8_t idx)
{
	return ((uint16_t)bMbRxBuf[3 + idx*2] << 8) | bMbRxBuf[4 + idx*2];
}

//폴링 응답 -> 무게 / 상태 저장
void fcPollResult(uint8_t res)
{
	uint8_t b,ch;

	b = bReqBoard;

	if(res == RES_OK)
	{
		for(ch=0;ch<BOARD_CH;ch++)
		{
			iWeight[b*BOARD_CH + ch] = (int16_t)fcMbGetReg(S_WEIGHT1 + ch);
		}
		bDetect[b]  = fcMbGetReg(S_DETECT) & 0x1f;
		wError[b]   = fcMbGetReg(S_ERROR);
		wBoardFw[b] = fcMbGetReg(S_FW_VERSION);
		bZeroErr[b] = fcMbGetReg(S_ZERO_ERR) & 0x1f;
		bFailCnt[b] = 0;
		bOnline[b]  = 1;
	}
	else if(res == RES_REJECT)
	{
		bFailCnt[b] = 0;		//응답은 있음
		bOnline[b]  = 1;
	}
	else
	{
		if(++bFailCnt[b] >= MB_FAIL_MAX)
		{
			bFailCnt[b] = MB_FAIL_MAX;
			bOnline[b] = 0;
		}
	}
}

void fcAckSend(uint8_t res);

//작업 결과 처리 -> 다음 작업 , 모두 끝나면 ACK
void fcJobNext(uint8_t res)
{
	if(res != RES_OK && bJobResult == RES_OK)	bJobResult = res;	//첫 번째 에러 유지

	bJobPhase = 0;
	bJobRetry = 0;

	if(++bJobIdx >= bJobNum)
	{
		fcAckSend(bJobResult);
		bJobNum = 0;
		bJobIdx = 0;
	}
}

void fcJobResult(uint8_t res)
{
	job_t *j = &stJob[bJobIdx];

	if(res == RES_COMM)		//무응답 -> 재시도
	{
		fcPollResult(res);

		if(++bJobRetry > MB_JOB_RETRY)	fcJobNext(RES_COMM);
		return;
	}

	bOnline[j->board] = 1;
	bFailCnt[j->board] = 0;

	if(res == RES_REJECT)
	{
		fcJobNext(RES_REJECT);
		return;
	}

	if(bJobPhase == 0)		//쓰기 완료
	{
		if(j->type == JOB_CMD)
		{
			wHoldCnt[j->board] = MB_HOLD_CMD;	//영점/스팬 처리 + EEPROM 저장 대기
			bJobPhase = 1;
			bJobRetry = 0;
		}
		else
		{
			wHoldCnt[j->board] = MB_HOLD_WRITE;
			fcJobNext(RES_OK);
		}
	}
	else					//결과 읽기 완료
	{
		if(fcMbGetReg(0) == 0)	fcJobNext(RES_OK);
		else					fcJobNext(RES_FAIL);
	}
}

void fcMbMaster(void)
{
	uint8_t k,b,res;
	job_t *j;

	if(bMbState == MS_WAIT)
	{
		b = bReqBoard;

		if(bMbRxDone)
		{
			res = fcMbCheck();
			bDbgLastLen[b] = bMbRxLen;
			if(res == RES_COMM){ if(wDbgBad[b] < 0xffff)	wDbgBad[b]++; }
			else			   { if(wDbgOk[b] < 0xffff)		wDbgOk[b]++; }
		}
		else if(bMbTxEnd && (wMbWaitCnt >= MB_RESP_TIMEOUT) && (bMbRxBusy==0))
		{
			res = RES_COMM;		//무응답
			bDbgLastLen[b] = 0;
			if(wDbgTimeout[b] < 0xffff)	wDbgTimeout[b]++;
		}
		else	return;

		bMbState = MS_IDLE;
		wMbGapCnt = 0;

		if(bReqType == 0)	fcPollResult(res);
		else				fcJobResult(res);
		return;
	}

	//MS_IDLE
	if(wMbGapCnt < MB_POLL_GAP)	return;

	//1. 상위 명령 작업
	if(bJobIdx < bJobNum)
	{
		j = &stJob[bJobIdx];

		if(wHoldCnt[j->board] == 0)
		{
			if(bJobPhase == 0)
			{
				bReqType = 1;
				fcMbWriteReq(j->board, j->reg, j->qty, j->val);
			}
			else
			{
				bReqType = 2;
				fcMbReadReq(j->board, S_CMD_RESULT, 1);
			}
			return;
		}
	}

	//2. 폴링 (대기중인 보드는 건너뜀)
	for(k=0;k<BOARD_NUM;k++)
	{
		b = bPollBoard;
		if(++bPollBoard >= BOARD_NUM)	bPollBoard = 0;

		if(wHoldCnt[b] == 0)
		{
			bReqType = 0;
			fcMbReadReq(b, POLL_REG_START, POLL_REG_NUM);
			return;
		}
	}
}

//================================================================
// RS232 상위 장치 송신
//================================================================
void fcPcPutc(char c)
{
	uint16_t next;

	next = wPcTxHead + 1;
	if(next >= PC_TX_SIZE)	next = 0;

	while(next == wPcTxTail)	wdt_reset();	//버퍼 가득 -> 대기

	cPcTxBuf[wPcTxHead] = c;
	wPcTxHead = next;
	UCSR0B |= (1<<UDRIE0);
}

//프레임 시작 '$'
void fcOutStart(void)
{
	bOutCs = 0;
	fcPcPutc('$');
}

//체크섬 포함 문자 출력
void fcOutChar(char c)
{
	bOutCs ^= c;
	fcPcPutc(c);
}

void fcOutStr(const char *s)
{
	while(*s)	fcOutChar(*s++);
}

void fcOutUint(uint16_t v)
{
	char buf[6];
	uint8_t n=0;

	do{
		buf[n++] = '0' + (v % 10);
		v /= 10;
	}while(v);

	while(n)	fcOutChar(buf[--n]);
}

//0.1 단위 정수 -> "-12.3"
void fcOutTenth(int16_t v)
{
	uint16_t u;

	if(v < 0){
		fcOutChar('-');
		u = (uint16_t)(-v);
	}
	else	u = v;

	fcOutUint(u / 10);
	fcOutChar('.');
	fcOutChar('0' + (u % 10));
}

//'*' + 체크섬 2자리 + CR LF
void fcOutEnd(void)
{
	const char hex[] = "0123456789ABCDEF";
	uint8_t cs = bOutCs;

	fcPcPutc('*');
	fcPcPutc(hex[cs >> 4]);
	fcPcPutc(hex[cs & 0x0f]);
	fcPcPutc('\r');
	fcPcPutc('\n');
}

//로드셀 상태 문자열 (LC_NUM 개)
void fcStatusMake(char *st)
{
	uint8_t b,ch,bit,n;

	for(b=0;b<BOARD_NUM;b++)
	{
		for(ch=0;ch<BOARD_CH;ch++)
		{
			n = b*BOARD_CH + ch;
			bit = 1<<ch;

			if(bOnline[b]==0)					st[n] = ST_COMM_ERR;
			else if(wError[b] & bit)			st[n] = ST_SENSOR_ERR;
			else if(bZeroErr[b] & bit)			st[n] = ST_ZERO_ERR;
			else if(wError[b] & (bit<<8))		st[n] = ST_OVER;
			else if(bDetect[b] & bit)			st[n] = ST_DETECT;
			else								st[n] = ST_NONE;
		}
	}
	st[LC_NUM] = 0;
}

//$DAT,상태10자리,무게1,...,무게10*CS
void fcSendData(void)
{
	char st[LC_NUM+1];
	uint8_t k,b;

	fcStatusMake(st);
	memcpy(cStatusOld, st, LC_NUM+1);

	fcOutStart();
	fcOutStr("DAT,");
	fcOutStr(st);

	for(k=0;k<LC_NUM;k++)
	{
		fcOutChar(',');
		b = k / BOARD_CH;
		if(bOnline[b])	fcOutTenth(iWeight[k]);
		else			fcOutTenth(0);
	}
	fcOutEnd();

	wAutoCnt = 0;
}

//자동 송신 : 주기마다 + 상태가 바뀌면 즉시
void fcAutoSend(void)
{
	char st[LC_NUM+1];

	if(wAutoTime == 0)	return;

	fcStatusMake(st);

	if((memcmp(st, cStatusOld, LC_NUM) != 0) || (wAutoCnt >= wAutoTime))
	{
		fcSendData();
	}
}

const char *fcResName(uint8_t res)
{
	if(res == RES_COMM)		return "COMM";
	if(res == RES_REJECT)	return "REJECT";
	if(res == RES_FAIL)		return "FAIL";
	return "OK";
}

//$ACK,CMD,OK  /  $ACK,CMD,NG,사유
void fcAckSend(uint8_t res)
{
	fcOutStart();
	fcOutStr("ACK,");
	fcOutStr(cJobName);
	if(res == RES_OK)	fcOutStr(",OK");
	else
	{
		fcOutStr(",NG,");
		fcOutStr(fcResName(res));
	}
	fcOutEnd();
}

void fcAckNg(const char *name, const char *reason)
{
	fcOutStart();
	fcOutStr("ACK,");
	fcOutStr(name);
	fcOutStr(",NG,");
	fcOutStr(reason);
	fcOutEnd();
}

void fcNak(const char *reason)
{
	fcOutStart();
	fcOutStr("NAK,");
	fcOutStr(reason);
	fcOutEnd();
}

//================================================================
// RS232 상위 장치 수신 명령
//================================================================
//정수 : 성공 1
uint8_t fcParseUint(const char *s, uint16_t *val)
{
	uint32_t v=0;

	if(*s == 0)	return 0;
	while(*s)
	{
		if(*s < '0' || *s > '9')	return 0;
		v = v*10 + (*s - '0');
		if(v > 65535)	return 0;
		s++;
	}
	*val = v;
	return 1;
}

//"12.3" / "12" -> 123 (0.1 단위) : 성공 1
uint8_t fcParseTenth(const char *s, uint16_t *val)
{
	uint32_t v=0;
	uint8_t dot=0,dec=0,digit=0;

	while(*s)
	{
		if(*s == '.')
		{
			if(dot)	return 0;
			dot = 1;
		}
		else if(*s >= '0' && *s <= '9')
		{
			if(dot)
			{
				if(dec == 0)	v = v*10 + (*s - '0');	//소수 첫째 자리까지만
				dec++;
			}
			else	v = v*10 + (*s - '0');
			digit = 1;
			if(v > 65535)	return 0;
		}
		else	return 0;
		s++;
	}
	if(!digit)	return 0;
	if(dec == 0)	v *= 10;
	if(v > 65535)	return 0;
	*val = v;
	return 1;
}

uint8_t fcHexVal(char c)
{
	if(c >= '0' && c <= '9')	return c - '0';
	if(c >= 'A' && c <= 'F')	return c - 'A' + 10;
	if(c >= 'a' && c <= 'f')	return c - 'a' + 10;
	return 0xff;
}

//작업 등록
void fcJobAdd(uint8_t board, uint8_t type, uint8_t reg, uint8_t qty, uint16_t v0, uint16_t v1)
{
	job_t *j;

	if(bJobNum >= JOB_MAX)	return;

	j = &stJob[bJobNum++];
	j->board  = board;
	j->type   = type;
	j->reg    = reg;
	j->qty    = qty;
	j->val[0] = v0;
	j->val[1] = v1;
}

void fcJobStart(const char *name)
{
	strncpy(cJobName, name, sizeof(cJobName)-1);
	cJobName[sizeof(cJobName)-1] = 0;
	bJobIdx = 0;
	bJobPhase = 0;
	bJobRetry = 0;
	bJobResult = RES_OK;
}

void fcPcProcess(void)
{
	char *arg[PC_ARG_MAX];
	char *p,*star;
	uint8_t argc=0,cs=0,k,b,n;
	uint16_t v1,v2,bits;

	//수신 라인 복사 후 다음 수신 허용
	cli();
	memcpy(cPcLine, (const char *)cPcRxBuf, PC_RX_SIZE);
	bPcRxLen = 0;
	bPcLineFg = 0;
	sei();

	if(cPcLine[0] != '$')	return;

	//체크섬 '*hh'
	star = strchr(cPcLine, '*');
	for(p=cPcLine+1; *p && p!=star; p++)	cs ^= *p;

	if(star)
	{
		if(fcHexVal(star[1])==0xff || fcHexVal(star[2])==0xff || (uint8_t)((fcHexVal(star[1])<<4)|fcHexVal(star[2])) != cs)
		{
			fcNak("CHECKSUM");
			return;
		}
		*star = 0;
	}
	else if(CHECKSUM_REQUIRED)
	{
		fcNak("CHECKSUM");
		return;
	}

	//대문자 변환 , ',' 로 분리
	for(p=cPcLine+1; *p; p++)
	{
		if(*p >= 'a' && *p <= 'z')	*p -= 0x20;
	}

	p = cPcLine + 1;
	arg[argc++] = p;
	while(*p && argc < PC_ARG_MAX)
	{
		if(*p == ',')
		{
			*p = 0;
			arg[argc++] = p + 1;
		}
		p++;
	}

	//$REQ : 데이터 1회 요청
	if(strcmp(arg[0], "REQ") == 0)
	{
		fcSendData();
	}
	//$DBG : 통신 진단 (보드별 정상,무응답,깨짐,마지막수신길이) 출력 후 카운터 초기화
	else if(strcmp(arg[0], "DBG") == 0)
	{
		fcOutStart();
		fcOutStr("DBG");
		for(b=0;b<BOARD_NUM;b++)
		{
			fcOutChar(',');	fcOutUint(wDbgOk[b]);
			fcOutChar(',');	fcOutUint(wDbgTimeout[b]);
			fcOutChar(',');	fcOutUint(wDbgBad[b]);
			fcOutChar(',');	fcOutUint(bDbgLastLen[b]);
			wDbgOk[b] = 0;
			wDbgTimeout[b] = 0;
			wDbgBad[b] = 0;
		}
		fcOutEnd();
	}
	//$VER : 버전
	else if(strcmp(arg[0], "VER") == 0)
	{
		fcOutStart();
		fcOutStr("VER,");
		fcOutUint(FW_VERSION);
		for(b=0;b<BOARD_NUM;b++)
		{
			fcOutChar(',');
			fcOutUint(bOnline[b] ? wBoardFw[b] : 0);
		}
		fcOutEnd();
	}
	//$AUTO,ms : 자동 송신 주기 (0 = 끔)
	else if(strcmp(arg[0], "AUTO") == 0)
	{
		if(argc < 2 || !fcParseUint(arg[1], &v1) || (v1 != 0 && (v1 < AUTO_TIME_MIN || v1 > AUTO_TIME_MAX)))
		{
			fcAckNg("AUTO", "PARAM");
			return;
		}
		wAutoTime = v1;
		EEPROM_WriteByte(ADDR_AUTO_TIME,   wAutoTime >> 8);
		EEPROM_WriteByte(ADDR_AUTO_TIME+1, wAutoTime & 0xff);

		fcOutStart();
		fcOutStr("ACK,AUTO,OK");
		fcOutEnd();
	}
	//보드에 쓰는 명령 (처리 결과는 ACK 로)
	else if(strcmp(arg[0], "ZERO") == 0 || strcmp(arg[0], "SPAN") == 0 || strcmp(arg[0], "LED") == 0 || strcmp(arg[0], "DTH") == 0)
	{
		if(bJobNum)		//이전 명령 처리중
		{
			fcAckNg(arg[0], "BUSY");
			return;
		}

		//$ZERO,n : n = 0 전체 , 1~10 로드셀 번호
		if(strcmp(arg[0], "ZERO") == 0)
		{
			if(argc < 2 || !fcParseUint(arg[1], &v1) || v1 > LC_NUM){ fcAckNg(arg[0], "PARAM"); return; }

			fcJobStart(arg[0]);
			if(v1 == 0)
			{
				for(b=0;b<BOARD_NUM;b++)	fcJobAdd(b, JOB_CMD, S_CMD, 1, S_CMD_ZERO_ALL, 0);
			}
			else
			{
				n = v1 - 1;
				fcJobAdd(n / BOARD_CH, JOB_CMD, S_CMD, 1, S_CMD_ZERO1 + (n % BOARD_CH), 0);
			}
		}
		//$SPAN,n,g : 분동 g(예 50.0) 올린 상태에서 스팬 교정 , n = 0 전체
		else if(strcmp(arg[0], "SPAN") == 0)
		{
			if(argc < 3 || !fcParseUint(arg[1], &v1) || v1 > LC_NUM ||
			   !fcParseTenth(arg[2], &v2) || v2 < CAL_WEIGHT_MIN || v2 > CAL_WEIGHT_MAX){ fcAckNg(arg[0], "PARAM"); return; }

			fcJobStart(arg[0]);
			if(v1 == 0)
			{
				for(b=0;b<BOARD_NUM;b++)	fcJobAdd(b, JOB_CMD, S_CAL_WEIGHT, 2, v2, S_CMD_SPAN_ALL);
			}
			else
			{
				n = v1 - 1;
				fcJobAdd(n / BOARD_CH, JOB_CMD, S_CAL_WEIGHT, 2, v2, S_CMD_SPAN1 + (n % BOARD_CH));
			}
		}
		//$LED,mode[,bits] : mode 0 = AUTO(감지시 ON) , 1 = MANUAL , bits = "1010000000" (LC1~10)
		else if(strcmp(arg[0], "LED") == 0)
		{
			if(argc < 2 || !fcParseUint(arg[1], &v1) || v1 > 1){ fcAckNg(arg[0], "PARAM"); return; }

			if(v1 == 1 && (argc < 3 || strlen(arg[2]) != LC_NUM)){ fcAckNg(arg[0], "PARAM"); return; }

			for(k=0;k<LC_NUM && v1==1;k++)
			{
				if(arg[2][k] != '0' && arg[2][k] != '1'){ fcAckNg(arg[0], "PARAM"); return; }
			}

			fcJobStart(arg[0]);
			for(b=0;b<BOARD_NUM;b++)
			{
				bits = 0;
				for(k=0;k<BOARD_CH && v1==1;k++)
				{
					if(arg[2][b*BOARD_CH + k] == '1')	bits |= (1<<k);
				}
				fcJobAdd(b, JOB_WRITE, S_LED_MODE, 2, v1, bits);
			}
		}
		//$DTH,on,off : 감지 기준 g (예 5.0,3.0)
		else
		{
			if(argc < 3 || !fcParseTenth(arg[1], &v1) || !fcParseTenth(arg[2], &v2) ||
			   v1 < DETECT_SET_MIN || v1 > DETECT_SET_MAX || v2 < DETECT_SET_MIN || v2 > v1){ fcAckNg(arg[0], "PARAM"); return; }

			fcJobStart(arg[0]);
			for(b=0;b<BOARD_NUM;b++)	fcJobAdd(b, JOB_WRITE, S_DETECT_ON, 2, v1, v2);
		}
	}
	else
	{
		fcNak("UNKNOWN");
	}
}

//================================================================
// EEPROM
//================================================================
void fcReadData(void)
{
	uint16_t temp;

	temp = EEPROM_ReadByte(ADDR_INIT);
	temp <<= 8;
	temp |= EEPROM_ReadByte(ADDR_INIT+1);

	if(temp != INIT_DATA)		//처음 전원 투입 -> 초기값 저장
	{
		EEPROM_WriteByte(ADDR_AUTO_TIME,   DEF_AUTO_TIME >> 8);
		EEPROM_WriteByte(ADDR_AUTO_TIME+1, DEF_AUTO_TIME & 0xff);
		EEPROM_WriteByte(ADDR_INIT,   INIT_DATA >> 8);
		EEPROM_WriteByte(ADDR_INIT+1, INIT_DATA & 0xff);
	}

	wAutoTime = EEPROM_ReadByte(ADDR_AUTO_TIME);
	wAutoTime <<= 8;
	wAutoTime |= EEPROM_ReadByte(ADDR_AUTO_TIME+1);

	if(wAutoTime != 0 && (wAutoTime < AUTO_TIME_MIN || wAutoTime > AUTO_TIME_MAX))	wAutoTime = DEF_AUTO_TIME;
}

void EEPROM_WriteByte(uint16_t Address, uint8_t Data)
{
	uint8_t sreg;

	if(EEPROM_ReadByte(Address) == Data)	return;	//같은 값이면 쓰지 않음

	while(EECR & (1<<EEPE));	// write가 완료될때까지 기다림

	EEAR = Address;	//주소 지정
	EEDR = Data;	//데이터 지정

	sreg = SREG;
	cli();						//EEMPE -> EEPE 4클럭 이내
	EECR |= (1<<EEMPE);
	EECR |= (1<<EEPE);
	SREG = sreg;
}

uint8_t EEPROM_ReadByte(uint16_t Address)
{
	while(EECR & (1<<EEPE));	// write가 완료될때까지 기다림

	EEAR = Address;				//주소 지정
	EECR |= (1<<EERE);			// 읽기 비트에 1 인가

	return EEDR;				// 읽은 데이터 값 Return
}
