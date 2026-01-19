/*
 * Usart.c
 *
 *  Created on: Feb 18, 2025
 *      Author: taegy
 */

#ifndef INC_USART_C_
#define INC_USART_C_

#include "main.h"

extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart2;

typedef enum {
	STEP_WAIT_FOR_COMMAND = 0,	// 0: 입/출고 위치 수신 대기
	STEP_MOVE_INIT_TO_LIFT,     // 1: 초기 위치 -> 리프트 이동
	STEP_WAIT_LIFT_UP,          // 2: 리프트 상승 대기
	STEP_MOVE_LIFT_TO_SLOT,     // 3: 리프트 -> 해당 칸 이동
	STEP_PERFORM_SLIDE,         // 4: 슬라이드 동작
	STEP_MOVE_SLOT_TO_LIFT,     // 5: 해당 칸 -> 리프트 이동
	STEP_WAIT_LIFT_DOWN,        // 6: 리프트 하강 대기
	STEP_MOVE_LIFT_TO_INIT,     // 7: 리프트 -> 초기 위치 이동
	STEP_OPERATION_COMPLETE,    // 8: 정상 종료 처리
	STEP_OPERATION_INCOMPLETE   // 9: 비정상 종료 or 오류 처리
} OperationStep;
extern OperationStep currentStep;		// 현재 전체 동작 상태 변수

typedef struct {
	uint8_t io_type;    // 입/출고
  uint8_t side;       // 선반 좌/우
  uint8_t floor;      // 층
  uint8_t cell;       // 칸
} Position_t;
extern Position_t currentPosition;	// 현재 입/출고 데이터

#define CELL_INIT 	210
#define CELL_LIFT 	153
#define CELL_1		101
#define CELL_2		54
#define CELL_3		9

#define COMMAND_MAX_SIZE 20									// 수신 최대 크기
extern uint8_t command[COMMAND_MAX_SIZE+2];	// 수신 받을 데이터
extern bool Rx_flag;												// 수신 완료

void UART_Transmit_EPS32(char *str);		// 데이터 송신
void UART_Receive_ESP32();							// 데이터 수신
void UART_Command_Vaildate(uint8_t length, OperationStep Step);	// 데이터 유효성 확인
void UART_Command_Parse();							// 데이터 분석

/*                                       Operation Data
 * ------------------------------------------------------------------------------------------
 * |             side = Left(0)	            	|             side = Right(1)				          |
 * |----------------------------------------	|-------------------------------------------  |
 * |   io_type = in(0)		io_type = out(1)	  |   io_type = in(0)		io_type = out(1)	      |
 * |----------------------------------------	|-------------------------------------------  |`
 * |   1-1 = 0x01			1-1 = 0x21			        | 	1-1 = 0x11			1-1 = 0x31			          |
 * |   1-2 = 0x02			1-2 = 0x22			        | 	1-2 = 0x12			1-2 = 0x32			          |
 * |   1-3 = 0x03			1-3 = 0x23			        | 	1-3 = 0x13			1-3 = 0x33			          |
 * |   2-1 = 0x04			2-1 = 0x24			        | 	2-1 = 0x14			2-1 = 0x34			          |
 * |   2-2 = 0x05			2-2 = 0x25			        | 	2-2 = 0x15			2-2 = 0x35			          |
 * |   2-3 = 0x06			2-3 = 0x26			        | 	2-3 = 0x16			2-3 = 0x36			          |
 * |   3-1 = 0x07			3-1 = 0x27			        | 	3-1 = 0x17			3-1 = 0x37			          |
 * |   3-2 = 0x08			3-2 = 0x28			        | 	3-2 = 0x18			3-2 = 0x38		            |
 * |   3-3 = 0x09			3-3 = 0x29			        | 	3-3 = 0x19			3-3 = 0x39			          |
 * |-----------------------------------------------------------------------------------------
 */

#endif /* INC_USART_C_ */
