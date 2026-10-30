#include "servo.h"

extern TIM_HandleTypeDef htim2;

void Servo_SetAngle(uint16_t angle)
{
    if(angle > SERVO_RANGE) angle = SERVO_RANGE;
    //if(angle < -SERVO_RANGE) angle = -SERVO_RANGE;

    uint16_t servoResolution = (2450 - 450)/180;
    uint16_t value = (uint16_t)(servoResolution * angle + 530);

    __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,value);
}

#if 0
void Motor_Servo_Task(void const * argument)
{
    // Enable cầu H
    HAL_GPIO_WritePin(R_EN_GPIO_Port, R_EN_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(L_EN_GPIO_Port, L_EN_Pin, GPIO_PIN_SET);

    for(;;) {
        Motor_SetSpeed(motorSpeed);   // motorSpeed do Encoder_Task + PID tính
        Servo_SetAngle(steerAngle);   // steerAngle do Communication_Task cập nhật
        osDelay(20); // 50Hz update rate
    }
}

#endif
