/*
 * STEP_MOTOR.h
 *
 *  Created on: May 20, 2025
 *      Author: taegy
 */

#ifndef STEP_MOTOR_B_MOTOR_H_
#define STEP_MOTOR_B_MOTOR_H_

#endif /* STEP_MOTOR_STEP_MOTOR_H_ */

#include "main.h"

extern TIM_HandleTypeDef htim9;		// STEP motor timer
extern TIM_HandleTypeDef htim5;		// Servo motor timer

#define STEP_DEGREE 0.036
#define GOAL_DEG  (360*2.6)

/* **************************************************** */
/*                    STEP VARIABLE                     */
/* **************************************************** */

// 스텝모터 제어용 구조체
typedef struct {
    uint32_t current_steps;  // 현재까지 진행한 스텝 수 (실제 이동 거리 반영)
    uint32_t total_steps;    // 목표로 하는 총 스텝 수 (이동해야 할 거리)
    bool step_flag;          // 1스텝 완료 여부를 나타내는 플래그 (true: 한 스텝 완료됨)
} StepMotor;

// 슬라이드의 전체 상태를 정의하는 열거형
typedef enum {
		SLIDE_STATE_INIT = 0,             	// 0. 슬라이드 관련 데이터 초기화
		SLIDE_STATE_MOVE_FORWARD,         	// 1. 1,2차 슬라이드 동작 시작 (전진)
		SLIDE_STATE_WAIT_FORWARD_DONE,    	// 2. 슬라이드 완료 대기 및 정지
		SLIDE_STATE_SERVO_ACTION,         	// 3. 서보 모터 동작
		SLIDE_STATE_MOVE_BACKWARD,       		// 4. 1,2차 슬라이드 동작 시작 (복귀)
		SLIDE_STATE_WAIT_BACKWARD_DONE,   	// 5. 슬라이드 완료 대기 및 정지
		SLIDE_STATE_DONE                  	// 6. 전체 슬라이드 동작 완료, 다음 단계로 전환
} Slide_State;

extern Slide_State slide_state;			// 현재 슬라이드 상태 변수


// 슬라이드 이동 방향 정의 (양쪽 선반 기준)
typedef enum {
    SLIDE_DIR_NONE,        // 방향 없음 또는 초기 상태
    SLIDE_DIR_LEFT,        // 왼쪽 선반 방향으로 슬라이드
    SLIDE_DIR_RIGHT        // 오른쪽 선반 방향으로 슬라이드
} SlideDirection;

// 슬라이드 입출고 모드를 정의 (시스템 제어 목적)
typedef enum {
    SLIDE_MODE_NONE,       // 입출고 모드 미설정 상태
    SLIDE_MODE_IN,         // 입고 작업 모드 (물건을 선반에 넣는 작업)
    SLIDE_MODE_OUT         // 출고 작업 모드 (물건을 선반에서 꺼내는 작업)
} SlideMode;

// 슬라이드 관련 전체 상태를 하나의 구조체로 통합 관리
typedef struct {
    SlideDirection direction;  // 슬라이드 방향 (LEFT, RIGHT 등)
    SlideMode mode;            // 슬라이드 입출고 모드 (IN, OUT 등)
    StepMotor motor;           // 해당 슬라이드를 구동하는 스텝모터 정보
} SlideControl;

extern SlideControl slide;

/* **************************************************** */
/*                    STEP MOTOR                        */
/* **************************************************** */

#define STEP_MOTOR_HOLDOFF_GPIO_Port 		GPIOE
#define STEP_MOTOR_HOLDOFF_Pin 					GPIO_PIN_2
#define STEP_MOTOR_DIR_GPIO_Port 				GPIOE
#define STEP_MOTOR_DIR_Pin 							GPIO_PIN_4

#define STEP_MOTOR_DIR_CW()							HAL_GPIO_WritePin(STEP_MOTOR_DIR_GPIO_Port, STEP_MOTOR_DIR_Pin,  GPIO_PIN_SET);
#define STEP_MOTOR_DIR_CCW()						HAL_GPIO_WritePin(STEP_MOTOR_DIR_GPIO_Port, STEP_MOTOR_DIR_Pin,  GPIO_PIN_RESET);
#define STEP_MOTOR_HOLDOFF_ON()					HAL_GPIO_WritePin(STEP_MOTOR_HOLDOFF_GPIO_Port, STEP_MOTOR_HOLDOFF_Pin,  GPIO_PIN_SET);
#define STEP_MOTOR_HOLDOFF_OFF()				HAL_GPIO_WritePin(STEP_MOTOR_HOLDOFF_GPIO_Port, STEP_MOTOR_HOLDOFF_Pin,  GPIO_PIN_RESET);

#define STEP_MOTOR_OC_GPIO_Port 				GPIOE
#define STEP_MOTOR_OC_Pin 							GPIO_PIN_5

/* **************************************************** */
/*                      FUNCTION                        */
/* **************************************************** */

void STEP_MOTOR_START();
void STEP_MOTOR_STOP();
void SERVO_UP();
void SERVO_DOWN();
