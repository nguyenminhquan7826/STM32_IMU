#include "communication.h"
#include "encoder.h"
#include "imu.h"
#include <assert.h>
#include <stdio.h>
PID_Handle_t pid;
int motorSpeed = 400, steerAngle = 75;
static CommandGuard command;
static int enc_ok = 1, imu_ok = 1, pwm, servo;
static uint32_t now;
uint32_t HAL_GetTick(void) { return now; }
void Control_GetCommand(CommandGuard *s) {
  CommandGuard_Tick(&command, now);
  *s = command;
}
void Control_Stop(uint8_t reason) { CommandGuard_Stop(&command, reason); }
uint8_t Encoder_GetLatest(Encoder_Data_t *s, uint32_t age) {
  (void)s;
  assert(age == 100);
  return enc_ok;
}
uint8_t IMU_GetLatest(BNO055_Data_t *s, uint32_t age) {
  assert(age == 100);
  s->system_status = 5;
  s->system_error = imu_ok ? 0 : 9;
  return 1;
}
void Motor_SetSpeed(uint16_t x) { pwm = x; }
void Servo_SetAngle(uint16_t x) { servo = x; }
int main(void) {
  CommandGuard_Init(&command);
  Control_ActuatorStep();
  assert(pwm == 0 && servo == 75);
  command.armed = 1;
  command.speed = .1;
  command.servo = 90;
  command.last_ms = 0;
  Control_ActuatorStep();
  assert(pwm == 400 && servo == 90);
  imu_ok = 0;
  Control_ActuatorStep();
  assert(!command.armed && pwm == 0);
  imu_ok = 1;
  command.armed = 1;
  command.speed = .1;
  enc_ok = 0;
  Control_ActuatorStep();
  assert(!command.armed && pwm == 0);
  enc_ok = 1;
  command.armed = 1;
  command.speed = .1;
  now = 300;
  Control_ActuatorStep();
  assert(pwm == 0 && !command.armed);
  now = 0;
  command.armed = 1;
  command.speed = 0;
  Control_ActuatorStep();
  assert(pwm == 0);
  puts("Actual actuator gate: disarm/zero, sensor error, stale encoder and "
       "watchdog PASS");
}
