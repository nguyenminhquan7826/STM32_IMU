#ifndef SERVO_H
#define SERVO_H

#include "main.h"
#include "cmsis_os.h"

#define SERVO_RANGE 180
void Servo_SetAngle(uint16_t angle);

#if 0
void Motor_Servo_Task(void const * argument);
#endif

#endif
