#include "communication.h"
#include "encoder.h"
#include "imu.h"
#include <assert.h>
#include <setjmp.h>
#include <stdio.h>
#include <string.h>
UART_HandleTypeDef huart2 = {USART2};
PID_Handle_t pid;
int steerAngle = 75, motorSpeed;
extern uint8_t rx_byte;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *);
static jmp_buf done;
static uint32_t now;
static const char *input;
static size_t offset;
static int pending;
static char replies[4096];
static size_t used;
static uint32_t limit = 100;
static int inject_overflow;
uint32_t HAL_GetTick(void) { return now; }
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *u, uint8_t *p,
                                      uint16_t n) {
  (void)u;
  (void)p;
  assert(n == 1);
  return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *u, uint8_t *p,
                                       uint16_t n) {
  (void)u;
  assert(used + n < sizeof(replies));
  memcpy(replies + used, p, n);
  used += n;
  replies[used] = 0;
  pending = 1;
  return HAL_OK;
}
uint8_t Encoder_GetLatest(Encoder_Data_t *s, uint32_t a) {
  (void)a;
  memset(s, 0, sizeof(*s));
  return 0;
}
uint8_t IMU_GetLatest(BNO055_Data_t *s, uint32_t a) {
  (void)a;
  memset(s, 0, sizeof(*s));
  return 0;
}
int32_t osDelay(uint32_t ms) {
  now += ms;
  if (pending) {
    pending = 0;
    HAL_UART_TxCpltCallback(&huart2);
  }
  if (now > limit)
    longjmp(done, 1);
  if (inject_overflow) {
    unsigned i;
    inject_overflow = 0;
    for (i = 0; i < 200; i++) {
      rx_byte = 'X';
      HAL_UART_RxCpltCallback(&huart2);
    }
    return 0;
  }
  /* One byte per task iteration exercises split commands and CRLF. */
  if (input[offset]) {
    rx_byte = (uint8_t)input[offset++];
    HAL_UART_RxCpltCallback(&huart2);
  }
  return 0;
}
static void frame(char *s, size_t n, const char *p) {
  snprintf(s, n, "%s,%04X\r\n", p, Command_CRC(p, strlen(p)));
}
int main(void) {
  char arm[96], run[96], stream[512];
  frame(arm, sizeof(arm), "CTL,1,1,ARM,0,75");
  frame(run, sizeof(run), "CTL,1,2,RUN,0.1,90");
  snprintf(stream, sizeof(stream), "%s%s", arm, run);
  input = stream;
  if (setjmp(done) == 0)
    Communication_Task(NULL);
  assert(g_command_guard.armed && g_command_guard.sequence == 2 &&
         g_command_guard.speed > .09f);
  assert(strstr(replies, "ACK,1,1,1,0,") && strstr(replies, "ACK,1,2,1,0,"));
  assert(pid.setpoint == 0);
  /* An oversized line containing a valid ARM suffix must not arm. */
  memset(stream, 'X', 140);
  snprintf(stream + 140, sizeof(stream) - 140, "%s%s", arm, run);
  now = 0;
  offset = 0;
  used = 0;
  pending = 0;
  limit = (uint32_t)strlen(stream) + 30U;
  if (setjmp(done) == 0)
    Communication_Task(NULL);
  assert(!g_command_guard.armed && g_command_guard.sequence == 0);
  /* A full RX ring loses framing: ignore through newline, then accept a
     complete subsequent command. No suffix recovery within a damaged line. */
  snprintf(stream, sizeof(stream), "%s%s%s", arm, arm, run);
  now = 0;
  offset = 0;
  used = 0;
  pending = 0;
  limit = (uint32_t)strlen(stream) + 30U;
  inject_overflow = 1;
  if (setjmp(done) == 0)
    Communication_Task(NULL);
  assert(g_command_guard.armed && g_command_guard.sequence == 2);
  puts("Actual UART split framing, ARM/RUN ACK, oversized line and ring "
       "overflow PASS");
}
