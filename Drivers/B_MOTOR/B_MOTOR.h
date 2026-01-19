/*
 * B_MOTOR.h
 *
 *  Created on: May 20, 2025
 *      Author: taegy
 */

#ifndef B_MOTOR_B_MOTOR_H_
#define B_MOTOR_B_MOTOR_H_

#endif /* B_MOTOR_B_MOTOR_H_ */

#include "main.h"

/* ******************************************************************************************************************* */
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim8;

/* ******************************************************************************************************************* */
typedef enum {
	ROBOT_STATE_INIT = 0,						// 0: 이동 관련 데이터 변수 초기화
	ROBOT_STATE_DISTANCE_CALC,			// 1: 목표 거리 계산(엔코더,(count) 거리 센서(cm))
	ROBOT_STATE_MOTOR_START,				// 2: 모터 구동 시작
	ROBOT_STATE_ENCODER_APPROACH,   // 3: 엔코더 기준 도착 여부 확인
	ROBOT_STATE_SENSOR_APPROACH,    // 4: 거리 센서 기준 도착 여부 확인
	ROBOT_STATE_DONE								// 5: 로봇 이동 동작 완료
} ROBOT_State;

extern ROBOT_State robot_state;	// 현재 로봇 이동 상태 변수

/* ******************************************************************************************************************* */
typedef enum {
	L_MOTOR,
	R_MOTOR
} MOTOR_Side;

typedef struct {
	uint16_t Encoder16New;
	uint16_t Encoder16Old;
	int16_t Encoder16Diff;

	int32_t Encoder32New;
	int32_t Encoder32Old;
	int32_t Encoder32Diff;
} MOTOR_Encoder_t;

typedef struct {
	int16_t Output16;
	int32_t Output32;

	int16_t Target16;
	int32_t Target32;
	int16_t Targetcm16;
	int32_t Targetcm32;

	int32_t  Error32New;
	int32_t  Error32Sum;
	int32_t  Error32Old;
	int32_t  Error32Diff;
	int32_t  Error32DiffOld;

	float Kp;
	float Ki;
	float Kd;

	float TermKp;
	float TermKi;
	float TermKd;

	float    f;
	int16_t  U;
	uint16_t  u;
} MOTOR_PID_t;

typedef struct {
	bool flag;
	MOTOR_Side Side;
	MOTOR_Encoder_t Encoder;
	MOTOR_PID_t PID;
} MOTOR_Type;

extern MOTOR_Type L_B_Motor;
extern MOTOR_Type R_B_Motor;
extern MOTOR_Type L_T_Motor;
extern MOTOR_Type R_T_Motor;

/* ******************************************************************************************************************* */
#define MOTOR_ENCODER_PPR    	380
#define MOTOR_MULTIPLE_RATIO  4
#define WHEEL_GEAR_RATIO      26

#define WHEEL_DIAMETER				10
#define WHEEL_CIRCUMFERENCE 	(WHEEL_DIAMETER * M_PI)

#define MOTOR_1REVOLUTION_ENCODER (MOTOR_ENCODER_PPR * MOTOR_MULTIPLE_RATIO)
#define WHEEL_AXIS_ENCODER    		(MOTOR_1REVOLUTION_ENCODER * WHEEL_GEAR_RATIO)
#define ENCODER_COUNTS_PER_CM 		(WHEEL_AXIS_ENCODER / WHEEL_CIRCUMFERENCE)

#define L_B_PWM_MAX 60
#define L_B_PWM_MIN 20
#define R_B_PWM_MAX 60
#define R_B_PWM_MIN 20

/* ******************************************************************************************************************* */

/* **************************************************** */
/*                     B L MOTOR                        */
/* **************************************************** */
#define B_MOTOR_L_EN_GPIO_Port 				GPIOD
#define B_MOTOR_L_EN_Pin 							GPIO_PIN_14
#define B_MOTOR_L_DIR_GPIO_Port 			GPIOD
#define B_MOTOR_L_DIR_Pin 						GPIO_PIN_15

#define B_MOTOR_L_PWM_GPIO_Port 			GPIOC
#define B_MOTOR_L_PWM_Pin 						GPIO_PIN_6

#define B_MOTOR_L_SIGNAL_A_GPIO_Port 	GPIOE
#define B_MOTOR_L_SIGNAL_A_Pin 				GPIO_PIN_9
#define B_MOTOR_L_SIGNAL_B_GPIO_Port 	GPIOE
#define B_MOTOR_L_SIGNAL_B_Pin 				GPIO_PIN_11

#define B_MOTOR_L_ENABLE()  					HAL_GPIO_WritePin(B_MOTOR_L_EN_GPIO_Port, B_MOTOR_L_EN_Pin,  GPIO_PIN_RESET)
#define B_MOTOR_L_DISABLE() 					HAL_GPIO_WritePin(B_MOTOR_L_EN_GPIO_Port, B_MOTOR_L_EN_Pin,  GPIO_PIN_SET)
#define B_MOTOR_L_DIR_CCW()   				HAL_GPIO_WritePin(B_MOTOR_L_DIR_GPIO_Port, B_MOTOR_L_DIR_Pin, GPIO_PIN_RESET)
#define B_MOTOR_L_DIR_CW()  					HAL_GPIO_WritePin(B_MOTOR_L_DIR_GPIO_Port, B_MOTOR_L_DIR_Pin, GPIO_PIN_SET)

#define B_MOTOR_L_READENCODER()				__HAL_TIM_GET_COUNTER(&htim1);

/* **************************************************** */
/*                     B R MOTOR                        */
/* **************************************************** */
#define B_MOTOR_R_EN_GPIO_Port 				GPIOA
#define B_MOTOR_R_EN_Pin 							GPIO_PIN_5
#define B_MOTOR_R_DIR_GPIO_Port 			GPIOA
#define B_MOTOR_R_DIR_Pin 						GPIO_PIN_6

#define B_MOTOR_R_PWM_GPIO_Port 			GPIOC
#define B_MOTOR_R_PWM_Pin 						GPIO_PIN_7

#define B_MOTOR_R_SIGNAL_A_GPIO_Port 	GPIOA
#define B_MOTOR_R_SIGNAL_A_Pin 				GPIO_PIN_15
#define B_MOTOR_R_SIGNAL_B_GPIO_Port 	GPIOB
#define B_MOTOR_R_SIGNAL_B_Pin 				GPIO_PIN_3

#define B_MOTOR_R_ENABLE()  					HAL_GPIO_WritePin(B_MOTOR_R_EN_GPIO_Port, B_MOTOR_R_EN_Pin,  GPIO_PIN_RESET)
#define B_MOTOR_R_DISABLE() 					HAL_GPIO_WritePin(B_MOTOR_R_EN_GPIO_Port, B_MOTOR_R_EN_Pin,  GPIO_PIN_SET)
#define B_MOTOR_R_DIR_CCW()   				HAL_GPIO_WritePin(B_MOTOR_R_DIR_GPIO_Port, B_MOTOR_R_DIR_Pin, GPIO_PIN_RESET)
#define B_MOTOR_R_DIR_CW()  					HAL_GPIO_WritePin(B_MOTOR_R_DIR_GPIO_Port, B_MOTOR_R_DIR_Pin, GPIO_PIN_SET)

#define B_MOTOR_R_READENCODER()				__HAL_TIM_GET_COUNTER(&htim2);

/* ******************************************************************************************************************* */
void B_MOTOR_Init();
void B_MOTOR_ReadEncoder();
uint16_t B_MOTOR_SETPWM(MOTOR_Type* motor, int16_t pulse);
void B_MOTOR_PidControl();

/* ******************************************************************************************************************* */

/* **************************************************** */
/*                     T L MOTOR                        */
/* **************************************************** */
#define T_MOTOR_L_EN_GPIO_Port 				GPIOE
#define T_MOTOR_L_EN_Pin 							GPIO_PIN_15
#define T_MOTOR_L_DIR_GPIO_Port 			GPIOE
#define T_MOTOR_L_DIR_Pin 						GPIO_PIN_14

#define T_MOTOR_L_PWM_GPIO_Port 			GPIOC
#define T_MOTOR_L_PWM_Pin 						GPIO_PIN_8

#define T_MOTOR_L_SIGNAL_A_GPIO_Port 	GPIOB
#define T_MOTOR_L_SIGNAL_A_Pin 				GPIO_PIN_4
#define T_MOTOR_L_SIGNAL_B_GPIO_Port 	GPIOB
#define T_MOTOR_L_SIGNAL_B_Pin 				GPIO_PIN_5

#define T_MOTOR_L_ENABLE()  					HAL_GPIO_WritePin(T_MOTOR_L_EN_GPIO_Port, T_MOTOR_L_EN_Pin,  GPIO_PIN_SET)
#define T_MOTOR_L_DISABLE() 					HAL_GPIO_WritePin(T_MOTOR_L_EN_GPIO_Port, T_MOTOR_L_EN_Pin,  GPIO_PIN_RESET)
#define T_MOTOR_L_DIR_CCW()   				HAL_GPIO_WritePin(T_MOTOR_L_DIR_GPIO_Port, T_MOTOR_L_DIR_Pin, GPIO_PIN_SET)
#define T_MOTOR_L_DIR_CW()  					HAL_GPIO_WritePin(T_MOTOR_L_DIR_GPIO_Port, T_MOTOR_L_DIR_Pin, GPIO_PIN_RESET)

#define T_MOTOR_L_READENCODER()				__HAL_TIM_GET_COUNTER(&htim3);

/* **************************************************** */
/*                     T R MOTOR                        */
/* **************************************************** */
#define T_MOTOR_R_CW_GPIO_Port 				GPIOE
#define T_MOTOR_R_CW_Pin 							GPIO_PIN_7
#define T_MOTOR_R_CCW_GPIO_Port 			GPIOE
#define T_MOTOR_R_CCW_Pin 						GPIO_PIN_8

#define T_MOTOR_R_PWM_GPIO_Port 			GPIOC
#define T_MOTOR_R_PWM_Pin 						GPIO_PIN_9

#define T_MOTOR_R_SIGNAL_A_GPIO_Port 	GPIOD
#define T_MOTOR_R_SIGNAL_A_Pin 				GPIO_PIN_12
#define T_MOTOR_R_SIGNAL_B_GPIO_Port 	GPIOD
#define T_MOTOR_R_SIGNAL_B_Pin 				GPIO_PIN_13

#define T_MOTOR_R_DISABLE() 					do { \
																				HAL_GPIO_WritePin(T_MOTOR_R_CW_GPIO_Port, T_MOTOR_R_CW_Pin,  GPIO_PIN_RESET);	\
																				HAL_GPIO_WritePin(T_MOTOR_R_CCW_GPIO_Port, T_MOTOR_R_CCW_Pin,  GPIO_PIN_RESET);	\
																			}while(0)
#define T_MOTOR_R_DIR_CW()   					do { \
																				HAL_GPIO_WritePin(T_MOTOR_R_CW_GPIO_Port, T_MOTOR_R_CW_Pin,  GPIO_PIN_SET);	\
																				HAL_GPIO_WritePin(T_MOTOR_R_CCW_GPIO_Port, T_MOTOR_R_CCW_Pin,  GPIO_PIN_RESET);	\
																			}while(0)
#define T_MOTOR_R_DIR_CCW()   				do { \
																				HAL_GPIO_WritePin(T_MOTOR_R_CW_GPIO_Port, T_MOTOR_R_CW_Pin,  GPIO_PIN_RESET);	\
																				HAL_GPIO_WritePin(T_MOTOR_R_CCW_GPIO_Port, T_MOTOR_R_CCW_Pin,  GPIO_PIN_SET);	\
																			}while(0)

#define T_MOTOR_R_READENCODER()				__HAL_TIM_GET_COUNTER(&htim4);

/* ******************************************************************************************************************* */

void T_MOTOR_Init();
void T_MOTOR_ReadEncoder();
void T_MOTOR_Control();


