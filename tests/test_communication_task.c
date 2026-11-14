#include "communication.h"
#include "encoder.h"
#include "imu.h"
#include "task.h"
#include <setjmp.h>
#include <assert.h>
#include <stdio.h>
#include <string.h>
UART_HandleTypeDef huart2={USART2};
PID_Handle_t pid;
int steerAngle=75,motorSpeed=0;
static uint32_t now;
static jmp_buf exit_task;
static char captured[384];
uint32_t HAL_GetTick(void){return now;}
int32_t osDelay(uint32_t ms){now+=ms;assert(now<100);return 0;}
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *u,uint8_t *b,uint16_t n){(void)u;(void)b;assert(n==1);return HAL_OK;}
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *u,uint8_t *b,uint16_t n){(void)u;assert(n<384);memcpy(captured,b,n);captured[n]=0;longjmp(exit_task,1);return HAL_OK;}
uint8_t Encoder_GetLatest(Encoder_Data_t *o,uint32_t age){assert(age==100);memset(o,0,sizeof(*o));o->sequence=10;o->timestamp_ms=now-1;o->total_ticks=17;o->delta_ticks=1;o->speed_mps=.1f;o->interval_ms=10;o->valid=1;return 1;}
uint8_t IMU_GetLatest(BNO055_Data_t *o,uint32_t age){assert(age==100);memset(o,0,sizeof(*o));o->sequence=20;o->timestamp_ms=now-5;o->read_start_ms=now-10;o->read_end_ms=now-5;o->gyro_x_dps=1;o->gyro_y_dps=2;o->gyro_z_dps=3;o->yaw_deg=10;o->roll_deg=4;o->pitch_deg=5;o->calibration_raw=0x34;o->system_status=5;o->valid=1;return 1;}
int main(void){
 if(setjmp(exit_task)==0)Communication_Task(NULL);
#ifdef TELEMETRY_PROTOCOL_VERSION
 assert(now==20);
#else
 assert(now==40);
#endif
 assert(g_uart_tx_drop_count==0&&g_uart_tx_truncate_count==0);
 fputs(captured,stdout);fprintf(stderr,"Actual communication task snapshot/format test PASS\n");
}
