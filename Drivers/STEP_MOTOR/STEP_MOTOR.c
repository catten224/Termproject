/*
 * STEP_MOTOR.c
 *
 *  Created on: May 20, 2025
 *      Author: taegy
 */

#include <STEP_MOTOR.h>

Slide_State slide_state = SLIDE_STATE_INIT;			// 현재 슬라이드 상태 변수
SlideControl slide = {
    .direction = SLIDE_DIR_NONE,    // 방향 없음
    .mode      = SLIDE_MODE_NONE,   // 모드 없음
    .motor     = {
        .current_steps = 0,                          // 시작 스텝 0
        .total_steps   = (GOAL_DEG) / STEP_DEGREE,   // 목표 각도 → 스텝 변환
        .step_flag     = false                       // 아직 스텝 완료 안 됨
    },
};

void STEP_MOTOR_START()
{
    slide.motor.current_steps = 0;                    // 스텝 카운터 초기화
    slide.motor.step_flag = false;                    // 스텝 완료 플래그 초기화
    HAL_TIM_OC_Start_IT(&htim9, TIM_CHANNEL_1);       // 타이머 인터럽트 기반 PWM 시작
}

void STEP_MOTOR_STOP()
{
    HAL_TIM_OC_Stop_IT(&htim9, TIM_CHANNEL_1);        // 타이머 인터럽트 정지 (모터 정지)
    slide.motor.step_flag = true;                     // 스텝 완료 표시 (논리 상태 업데이트용)
}

void SERVO_UP()
{
	__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 6);
	__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, 6);
}

void SERVO_DOWN()
{
	__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 2);
	__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, 10);
}
