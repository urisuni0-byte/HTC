/*
 * define.h : 메인보드 (ATmega2560 16MHz)
 *  RS485 (Modbus RTU 마스터) <-> 로드셀 컨트롤 보드 SVROCB x 2
 *  RS232 (ASCII 프로토콜)   <-> 상위 장치 (PC / HMI)
 */
//261112
//261117 RS485 USART3 , 영점 에러 'Z'

#ifndef DEFINE_H_
#define DEFINE_H_

#define			FW_VERSION			101		//v1.01

//================================================================
// 포트  *** 메인보드 회로도가 없어서 임의로 정함. 회로도에 맞게 수정 ***
//================================================================
// USART0 (PE0 RXD0 / PE1 TXD0) : RS232 상위 장치 (MAX232 등)
// USART3 (PJ0 RXD3 / PJ1 TXD3) : RS485 (MAX485 RO / DI)
// PA0                          : RS485 DE,/RE  HIGH = 송신
// PB7                          : STATE LED     HIGH = ON

#define			RS485_DDR_INIT		(DDRA |= (1<<PA0))
#define			RS485_TX_MODE		(PORTA |=  (1<<PA0))
#define			RS485_RX_MODE		(PORTA &= ~(1<<PA0))

#define			LED_DDR_INIT		(DDRB |= (1<<PB7))
#define			LED_ON				(PORTB |=  (1<<PB7))
#define			LED_OFF				(PORTB &= ~(1<<PB7))

//================================================================
// 로드셀 보드
//================================================================
#define			BOARD_NUM			2		//컨트롤 보드 수 (DIP 0 -> 주소 1 , DIP 1 -> 주소 2)
#define			BOARD_CH			5		//보드당 로드셀
#define			LC_NUM				(BOARD_NUM*BOARD_CH)	//로드셀 1~10
#define			BOARD_ADDR_BASE		1		//보드 주소 = 보드번호(0~) + 1

//================================================================
// RS485 : Modbus RTU 마스터 (SVROCB 와 동일 설정)
//================================================================
#define			MB_BAUD				9600UL
#define			MB_UBRR				((F_CPU / (8UL * MB_BAUD)) - 1)	//U2X=1 -> 207
#define			MB_FRAME_GAP		5		//ms 수신 없으면 프레임 끝
#define			MB_RESP_TIMEOUT		100		//ms 요청 송신 완료 후 응답 대기
#define			MB_POLL_GAP			10		//ms 프레임 사이 간격
#define			MB_FAIL_MAX			3		//연속 3회 무응답 -> 보드 통신 끊김
#define			MB_JOB_RETRY		1		//쓰기 명령 재시도 횟수
#define			MB_HOLD_WRITE		300		//ms 설정 쓰기 후 해당 보드 대기 (보드 EEPROM 저장)
#define			MB_HOLD_CMD			500		//ms 영점/스팬 명령 후 결과 읽기 전 대기
#define			MB_RX_SIZE			64
#define			MB_TX_SIZE			32

#define			POLL_REG_START		0		//주기 읽기 : 0~15 (무게, 감지, 에러, ID, FW, raw, 영점 에러)
#define			POLL_REG_NUM		16		//0~15 (15 = 영점 에러)

//SVROCB 레지스터 (SVROCB_FW/define.h 와 동일)
#define			S_WEIGHT1			0		//0~4 무게 0.1g
#define			S_DETECT			5		//bit0~4 감지
#define			S_ERROR				6		//bit0~4 센서 이상 , bit8~12 과부하
#define			S_BOARD_ID			7
#define			S_FW_VERSION		9
#define			S_ZERO_ERR			15		//bit0~4 전원 투입 영점 에러
#define			S_LED_MODE			20
#define			S_LED_MANUAL		21
#define			S_DETECT_ON			22
#define			S_DETECT_OFF		23
#define			S_CAL_WEIGHT		24
#define			S_CMD				25
#define			S_CMD_RESULT		26

#define			S_CMD_ZERO1			1		//1~5
#define			S_CMD_ZERO_ALL		9
#define			S_CMD_SPAN1			11		//11~15
#define			S_CMD_SPAN_ALL		19

//보드 명령 작업
#define			JOB_WRITE			0		//쓰기만
#define			JOB_CMD				1		//쓰기 -> 대기 -> 결과(26) 읽기
#define			JOB_MAX				BOARD_NUM

//작업 결과
#define			RES_OK				0
#define			RES_COMM			1		//보드 무응답
#define			RES_REJECT			2		//보드가 값 거부 (Modbus 예외)
#define			RES_FAIL			3		//영점/스팬 실패 (센서 이상, 변화량 부족)

//마스터 상태
#define			MS_IDLE				0
#define			MS_WAIT				1

//================================================================
// RS232 : 상위 장치 ASCII 프로토콜  $CMD,data,...*CS<CR><LF>
//================================================================
#define			PC_BAUD				115200UL
#define			PC_UBRR				((F_CPU / (8UL * PC_BAUD)) - 1)	//U2X=1 -> 16 (2.1%)
#define			PC_RX_SIZE			80
#define			PC_TX_SIZE			512
#define			PC_ARG_MAX			6
#define			CHECKSUM_REQUIRED	0		//0 = *CS 없어도 받음 (터미널 시험용) , 1 = 필수

#define			DEF_AUTO_TIME		200		//ms 자동 송신 주기 (0 = 요청시만)
#define			AUTO_TIME_MIN		100
#define			AUTO_TIME_MAX		10000

#define			DETECT_SET_MIN		1		//0.1g
#define			DETECT_SET_MAX		1000	//100.0g
#define			CAL_WEIGHT_MIN		100		//10.0g
#define			CAL_WEIGHT_MAX		1200	//120.0g

//로드셀 상태 문자
#define			ST_NONE				'0'		//없음
#define			ST_DETECT			'1'		//있음
#define			ST_OVER				'O'		//있음 + 과부하
#define			ST_SENSOR_ERR		'E'		//센서 이상
#define			ST_COMM_ERR			'C'		//보드 통신 끊김
#define			ST_ZERO_ERR			'Z'		//전원 투입 영점 에러 (무게 측정 안함)

//================================================================
// EEPROM
//================================================================
#define			INIT_DATA			0xA55A
#define			ADDR_INIT			0		//2 byte
#define			ADDR_AUTO_TIME		2		//2 byte

#endif /* DEFINE_H_ */
