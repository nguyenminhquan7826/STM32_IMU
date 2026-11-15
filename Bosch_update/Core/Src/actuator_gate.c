#include "communication.h"
#include "encoder.h"
#include "imu.h"
#include "motor.h"
#include "servo.h"
#include "task.h"
extern int motorSpeed, steerAngle;
void Control_ActuatorStep(void) {
  CommandGuard command;
  Encoder_Data_t encoder;
  BNO055_Data_t imu;
  int pwm;
  /* Avoid a STOP preemption between command snapshot and PWM write. */
  taskENTER_CRITICAL();
  Control_GetCommand(&command);
  if (command.armed &&
      (!Encoder_GetLatest(&encoder, 100U) || !IMU_GetLatest(&imu, 100U) ||
       imu.system_status != 5U || imu.system_error != 0U)) {
    Control_Stop(COMMAND_SENSOR);
    command.armed = 0;
    command.speed = 0;
    command.servo = COMMAND_SERVO_CENTER;
  }
  pwm = (command.armed && command.speed > 0) ? motorSpeed : 0;
  if (pwm < 0)
    pwm = 0;
  if (pwm > MOTOR_MAX_INPUT)
    pwm = MOTOR_MAX_INPUT;
  steerAngle = command.armed ? command.servo : COMMAND_SERVO_CENTER;
  Motor_SetSpeed((uint16_t)pwm);
  Servo_SetAngle((uint16_t)steerAngle);
  taskEXIT_CRITICAL();
}
