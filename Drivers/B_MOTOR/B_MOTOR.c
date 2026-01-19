/*
 * B_MOTOR.c
 *
 *  Created on: May 20, 2025
 *      Author: taegy
 */

#include <B_MOTOR.h>

MOTOR_Type L_B_Motor = {
		.flag = false,
    .Side = L_MOTOR,
    .Encoder = {0},
    .PID = {
    		.Kp = 0.001f,
				.Ki = 0,
				.Kd = 0,
    }
};

MOTOR_Type R_B_Motor = {
		.flag = false,
    .Side = R_MOTOR,
    .Encoder = {0},
    .PID = {
    		.Kp = 0.001f,
				.Ki = 0,
				.Kd = 0,
    }
};

MOTOR_Type L_T_Motor = {
		.flag = false,
    .Side = L_MOTOR,
    .Encoder = {0},
};

MOTOR_Type R_T_Motor = {
		.flag = false,
    .Side = R_MOTOR,
    .Encoder = {0},
};

ROBOT_State robot_state = ROBOT_STATE_INIT;

/* ******************************************************************************************************************* */
void B_MOTOR_Init()
{
	// 목표 도착 플래그 초기화
	L_B_Motor.flag = false;
	R_B_Motor.flag = false;

	// 모터 초기화
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0);
	B_MOTOR_L_DISABLE();
	B_MOTOR_R_DISABLE();

	// 엔코더 초기화
	__HAL_TIM_SET_COUNTER(&htim1, 0);
	__HAL_TIM_SET_COUNTER(&htim2, 0);
	L_B_Motor.Encoder = (MOTOR_Encoder_t){0};
	R_B_Motor.Encoder = (MOTOR_Encoder_t){0};
}

void B_MOTOR_ReadEncoder()
{
	MOTOR_Encoder_t* encL = &L_B_Motor.Encoder;
	MOTOR_Encoder_t* encR = &R_B_Motor.Encoder;

	encL->Encoder16New = B_MOTOR_L_READENCODER();
	encR->Encoder16New = B_MOTOR_R_READENCODER();

	encL->Encoder16Diff = (int16_t)(encL->Encoder16New - encL->Encoder16Old);
	encR->Encoder16Diff = (int16_t)(encR->Encoder16New - encR->Encoder16Old);
	encL->Encoder32New += encL->Encoder16Diff;
	encR->Encoder32New += encR->Encoder16Diff;
	encL->Encoder32Diff = encL->Encoder32New - encL->Encoder32Old;
	encR->Encoder32Diff = encR->Encoder32New - encR->Encoder32Old;

	encL->Encoder16Old = encL->Encoder16New;
	encR->Encoder16Old = encR->Encoder16New;
	encL->Encoder32Old = encL->Encoder32New;
	encR->Encoder32Old = encR->Encoder32New;
}

uint16_t B_MOTOR_SETPWM(MOTOR_Type* motor, int16_t pulse){
	if(pulse<0) pulse = -pulse;

	if(motor->Side == L_MOTOR){
		if(pulse > L_B_PWM_MAX) pulse = L_B_PWM_MAX;
		if(pulse < L_B_PWM_MIN) pulse = L_B_PWM_MIN;
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, pulse);
	}
	else if(motor->Side == R_MOTOR){
		if(pulse > R_B_PWM_MAX) pulse = R_B_PWM_MAX;
		if(pulse < R_B_PWM_MIN) pulse = R_B_PWM_MIN;
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, pulse);
	}

	return pulse;
}

void B_MOTOR_PidControl()
{
	MOTOR_Encoder_t* encL = &L_B_Motor.Encoder;
	MOTOR_Encoder_t* encR = &R_B_Motor.Encoder;
	MOTOR_PID_t* pidL = &L_B_Motor.PID;
	MOTOR_PID_t* pidR = &R_B_Motor.PID;

	// 도달한 바퀴는 제어 중지
	if (!L_B_Motor.flag) {
			pidL->Error32New = pidL->Target32 - encL->Encoder32New;
			if (abs(encL->Encoder32New) > abs(pidL->Target32)) {
				L_B_Motor.flag = true;
				B_MOTOR_L_DISABLE();
			} else {
				pidL->TermKp = pidL->Kp * pidL->Error32New;
				pidL->f = pidL->TermKp;
				pidL->U = (int16_t)(pidL->f);
			}
	}

	if (!R_B_Motor.flag) {
			pidR->Error32New = pidR->Target32 - encR->Encoder32New;
			if (abs(encR->Encoder32New) > abs(pidR->Target32)) {
				R_B_Motor.flag = true;
				B_MOTOR_R_DISABLE();
			} else {
				pidR->TermKp = pidR->Kp * pidR->Error32New;
				pidR->f = pidR->TermKp;
				pidR->U = (int16_t)(pidR->f);
			}
	}

	if (pidL->U > 140) pidL->U = 140;
	if (pidR->U > 140) pidR->U = 140;


	pidL->u = B_MOTOR_SETPWM(&L_B_Motor, pidL->U);
	pidR->u = B_MOTOR_SETPWM(&R_B_Motor, pidR->U);
}
/* ******************************************************************************************************************* */

void T_MOTOR_Init()
{
	// 목표 도착 플래그 초기화
	L_T_Motor.flag = false;
	R_T_Motor.flag = false;

	// 모터 초기화
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 0);
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, 0);
	T_MOTOR_L_DISABLE();
	T_MOTOR_R_DISABLE();

	// 엔코더 초기화
	__HAL_TIM_SET_COUNTER(&htim3, 0);
	__HAL_TIM_SET_COUNTER(&htim4, 0);
	L_T_Motor.Encoder = (MOTOR_Encoder_t){0};
	R_T_Motor.Encoder = (MOTOR_Encoder_t){0};
}

void T_MOTOR_ReadEncoder()
{
	MOTOR_Encoder_t* encL = &L_T_Motor.Encoder;
	MOTOR_Encoder_t* encR = &R_T_Motor.Encoder;

	encL->Encoder16New = T_MOTOR_L_READENCODER();
	encR->Encoder16New = T_MOTOR_R_READENCODER();

	encL->Encoder16Diff = (int16_t)(encL->Encoder16New - encL->Encoder16Old);
	encR->Encoder16Diff = (int16_t)(encR->Encoder16New - encR->Encoder16Old);
	encL->Encoder32New += encL->Encoder16Diff;
	encR->Encoder32New += encR->Encoder16Diff;
	encL->Encoder32Diff = encL->Encoder32New - encL->Encoder32Old;
	encR->Encoder32Diff = encR->Encoder32New - encR->Encoder32Old;

	encL->Encoder16Old = encL->Encoder16New;
	encR->Encoder16Old = encR->Encoder16New;
	encL->Encoder32Old = encL->Encoder32New;
	encR->Encoder32Old = encR->Encoder32New;
}

void T_MOTOR_Control()
{
  MOTOR_Encoder_t* encL = &L_T_Motor.Encoder;
  MOTOR_Encoder_t* encR = &R_T_Motor.Encoder;
  MOTOR_PID_t* pidL = &L_T_Motor.PID;
  MOTOR_PID_t* pidR = &R_T_Motor.PID;

  // 도달한 바퀴는 제어 중지
  if (!L_T_Motor.flag) {
      pidL->Error32New = pidL->Target32 - encL->Encoder32New;
      if (abs(encL->Encoder32New) > abs(pidL->Target32)) {
      	L_T_Motor.flag = true;
        T_MOTOR_L_DISABLE();
      }
  }

  if (!R_T_Motor.flag) {
      pidR->Error32New = pidR->Target32 - encR->Encoder32New;
      if (abs(encR->Encoder32New) > abs(pidR->Target32)) {
      	R_T_Motor.flag = true;
        T_MOTOR_R_DISABLE();
      }
  }
}
