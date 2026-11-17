/*
 * define.h : SVROCB_V1 로드셀 컨트롤 보드 (ATmega8A-AU 16MHz)
 *
 */
//261112

#ifndef DEFINE_H_
#define DEFINE_H_

#define			FW_VERSION			100		//v1.00

//================================================================
// 포트 (회로도 SVROCB_V1 기준)
//================================================================
// PC0~PC4 (ADC0~4) : L/C_ADC1~5  (INA155 x50 -> MCP6242 x11)
// PC5              : STATE LED D1  (LOW = ON)
// PD0 / PD1        : RS485_RxD / RS485_TxD  (MAX485 RO / DI)
// PD2              : RS485_Drive (MAX485 DE,/RE)  HIGH = 송신
// PD3,PD4,PD5,PD6,PD7 : L/C_1ST ~ L/C_5ST LED D2~D6 (LOW = ON)
// PB0~PB2          : COM_ID_1~3 (DIP S/W J10, ON = LOW, 외부 10K 풀업)
// SW2 (Tact)       : /RESET 에 연결 -> 누르면 리셋 (전원 투입 영점 다시 실행)

//STATE LED (D1)
#define			LED_ON				(PORTC &= ~(1<<PC5))
#define			LED_OFF				(PORTC |=  (1<<PC5))

//스테이션 LED (D2~D6)
#define			ST1_LED_ON			(PORTD &= ~(1<<PD3))
#define			ST1_LED_OFF			(PORTD |=  (1<<PD3))
#define			ST2_LED_ON			(PORTD &= ~(1<<PD4))
#define			ST2_LED_OFF			(PORTD |=  (1<<PD4))
#define			ST3_LED_ON			(PORTD &= ~(1<<PD5))
#define			ST3_LED_OFF			(PORTD |=  (1<<PD5))
#define			ST4_LED_ON			(PORTD &= ~(1<<PD6))
#define			ST4_LED_OFF			(PORTD |=  (1<<PD6))
#define			ST5_LED_ON			(PORTD &= ~(1<<PD7))
#define			ST5_LED_OFF			(PORTD |=  (1<<PD7))

//RS485 방향
#define			RS485_TX_MODE		(PORTD |=  (1<<PD2))
#define			RS485_RX_MODE		(PORTD &= ~(1<<PD2))

//DIP S/W (ON = 0) -> 0~7
#define			IN_DIP_ID			((~PINB) & 0x07)

//================================================================
// 로드셀 / ADC
//================================================================
#define			CH_NUM				5		//보드당 로드셀 5채널
#define			ADC_AVG_CNT			50		//2ms x 50 = 100ms 동안 합산 (raw = 0~51150)
#define			FILTER_NUM			8		//100ms x 8 = 800ms 이동평균

#define			RAW_ERR_LOW			(5UL*ADC_AVG_CNT)		//ADC 5 이하   -> 센서 이상(단선/포화)
#define			RAW_ERR_HIGH		(1018UL*ADC_AVG_CNT)	//ADC 1018 이상 -> 센서 이상
#define			SENSOR_ERR_TIME		10		//x100ms = 1초 연속이면 에러 판단/해제

//무게 단위 : 0.1g
#define			SPAN_REF_WEIGHT		1000	//스팬은 100.0g 기준 raw 값으로 저장
#define			DEF_SPAN			26710L	//초기 스팬 : TAL221 , 2단 R2=30K/R3=1K , R4=1K/R6=30K -> x30 (실측 x30.9 일때 27520 기준 환산)
#define			SPAN_MIN_RAW		250		//스팬 교정 최소 변화량 (ADC 5 x 50)
#define			OVER_WEIGHT			1200	//120.0g 이상 -> 과부하
#define			WEIGHT_LIMIT		32000	//표시 제한

#define			DEF_DETECT_ON		50		//5.0g 이상 -> 감지 (LED ON)
#define			DEF_DETECT_OFF		30		//3.0g 미만 -> 감지 해제
#define			DETECT_MAX			1000	//설정 범위 0.1~100.0g
#define			DEF_CAL_WEIGHT		1000	//스팬 교정 분동 100.0g
#define			CAL_WEIGHT_MIN		100		//10.0g
#define			CAL_WEIGHT_MAX		1200	//120.0g

//전원 투입 영점 교정 (전원 ON 시 로드셀 무부하로 가정)
// 저장된 영점이 없으면 현재값을 영점으로 EEPROM 저장
// 저장 영점 기준 ±ZERO_ERR_RANGE 이내 -> 현재값을 새 영점으로 EEPROM 저장
// 범위 밖 -> 영점 에러 : 해당 채널 LED 점멸 , 무게 측정 안함 (0) , 저장 영점은 그대로 유지
//          해제 : 무부하로 다시 전원 ON , 또는 메인보드 영점 명령(CMD 1~5, 9)
#define			POWERON_ZERO_TIME	30		//x100ms = 3초 후 실행 (이동평균 안정 대기)
#define			ZERO_ERR_RANGE		50		//±5.0g (빈병 15g 이 올라간 채로 켜면 에러)
#define			ZERO_UNSET_L		0x0000	//EEPROM 영점 미설정 값
#define			ZERO_UNSET_H		0xFFFF

#define			LED_MODE_AUTO		0		//무게 감지시 스테이션 LED ON
#define			LED_MODE_MANUAL		1		//메인보드가 LED 직접 제어

//================================================================
// 통신 : 메인보드(마스터) <-> 컨트롤보드(슬레이브)  Modbus RTU
//================================================================
#define			MB_BAUD				9600UL
#define			MB_UBRR				((F_CPU / (8UL * MB_BAUD)) - 1)	//U2X=1 , 9600 -> 207 (0.2%)
#define			MB_FRAME_GAP		5		//ms 수신 없으면 프레임 끝 (3.5문자 = 3.65ms @9600)
#define			MB_TX_DELAY			2		//ms 수신 후 응답 지연 (마스터 방향전환 시간)
#define			MB_ADDR_BASE		1		//슬레이브 주소 = DIP + 1  (DIP 0 -> 1, DIP 1 -> 2 ...)
#define			MB_RX_SIZE			64
#define			MB_TX_SIZE			72
#define			MB_READ_MAX			32		//FC03/04 한번에 읽기 최대 개수
#define			COMM_TIMEOUT		50		//x100ms = 5초 수신 없으면 통신 끊김

#define			MB_EX_FUNC			0x01
#define			MB_EX_ADDR			0x02
#define			MB_EX_VALUE			0x03

//Modbus 레지스터 맵 (FC03/FC04 읽기, FC06/FC16 쓰기)
#define			MB_REG_NUM			40

#define			M_WEIGHT1			0		//0~4  무게 CH1~5 (0.1g, signed)
#define			M_DETECT			5		//bit0~4 감지
#define			M_ERROR				6		//bit0~4 센서 이상, bit8~12 과부하
#define			M_BOARD_ID			7		//DIP 값 0~7
#define			M_LC_START_NO		8		//CH1 의 로드셀 번호 (DIP0 -> 1, DIP1 -> 6)
#define			M_FW_VERSION		9
#define			M_RAW1				10		//10~14 raw (0~51150)
#define			M_ZERO_ERR			15		//bit0~4 전원 투입 영점 에러

#define			M_LED_MODE			20		//쓰기 0 = AUTO, 1 = MANUAL
#define			M_LED_MANUAL		21		//쓰기 bit0~4 스테이션 LED (MANUAL)
#define			M_DETECT_ON			22		//쓰기 감지 ON  (0.1g)
#define			M_DETECT_OFF		23		//쓰기 감지 OFF (0.1g)
#define			M_CAL_WEIGHT		24		//쓰기 스팬 교정 분동 무게 (0.1g)
#define			M_CMD				25		//쓰기 명령 (처리 후 0)
#define			M_CMD_RESULT		26		//명령 결과

#define			M_ZERO1				30		//30~34 영점 raw
#define			M_SPAN1				35		//35~39 스팬 raw (100.0g 기준)

#define			M_W_START			M_LED_MODE	//쓰기 가능 영역
#define			M_W_END				M_CMD

//명령 (M_CMD)
#define			CMD_NONE			0
#define			CMD_ZERO1			1		//1~5 CH 영점
#define			CMD_ZERO_ALL		9		//전체 영점
#define			CMD_SPAN1			11		//11~15 CH 스팬 (CAL_WEIGHT 분동 올린 상태)
#define			CMD_SPAN_ALL		19		//전체 스팬
#define			CMD_CAL_INIT		90		//영점/스팬/설정 초기화

#define			CMD_RES_OK			0
#define			CMD_RES_FAIL		1		//센서 이상 또는 변화량 부족
#define			CMD_RES_UNKNOWN		2		//알 수 없는 명령

//================================================================
// EEPROM (ATmega8 : 512 byte)
//================================================================
#define			INIT_DATA			0x5AA5
#define			ADDR_INIT_H			0
#define			ADDR_INIT_L			1
#define			ADDR_ZERO			2		//5 x 2 byte
#define			ADDR_SPAN			12		//5 x 4 byte
#define			ADDR_DETECT_ON		32
#define			ADDR_DETECT_OFF		34
#define			ADDR_CAL_WEIGHT		36
#define			ADDR_LED_MODE		38

//bSaveFg
#define			SAVE_ZERO			0x01
#define			SAVE_SPAN			0x02
#define			SAVE_SET			0x04

#endif /* DEFINE_H_ */
