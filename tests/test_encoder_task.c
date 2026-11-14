#include "encoder.h"
#include "task.h"
#include <setjmp.h>
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
static TIM_TypeDef timer;
TIM_HandleTypeDef htim3;
PID_Handle_t pid;
int motorSpeed;
static jmp_buf exit_task;
static HAL_StatusTypeDef start_status;
static uint32_t now,interval;
static int waits,pid_calls;
static float pid_dt;
uint32_t HAL_GetTick(void){return now;}
TickType_t xTaskGetTickCount(void){return now;}
HAL_StatusTypeDef HAL_TIM_Encoder_Start(TIM_HandleTypeDef *h,uint32_t channel){assert(h==&htim3&&channel==TIM_CHANNEL_ALL);return start_status;}
int32_t osDelay(uint32_t ms){(void)ms;longjmp(exit_task,1);return 0;}
void vTaskDelayUntil(TickType_t *last,TickType_t dt){(void)last;(void)dt;if(waits++)longjmp(exit_task,1);now+=interval;timer.CNT=10;}
float PID_Update(PID_Handle_t *p,float speed,float dt){(void)p;(void)speed;pid_calls++;pid_dt=dt;return 123;}
static void reset(void){memset((void*)&g_encoder_latest,0,sizeof(g_encoder_latest));memset((void*)&g_encoder_task_status,0,sizeof(g_encoder_task_status));timer.CNT=0;htim3.Instance=&timer;now=100;interval=20;waits=pid_calls=0;pid_dt=0;motorSpeed=999;}
int main(void){
 reset();start_status=HAL_ERROR;if(setjmp(exit_task)==0)Encoder_Task(NULL);
 assert(!g_encoder_latest.valid&&!g_encoder_task_status.initialized&&g_encoder_task_status.start_status==HAL_ERROR&&motorSpeed==0&&pid_calls==0);
 reset();start_status=HAL_OK;if(setjmp(exit_task)==0)Encoder_Task(NULL);
 assert(g_encoder_latest.valid&&g_encoder_task_status.initialized&&g_encoder_latest.interval_ms==20&&g_encoder_latest.total_ticks==10);
 assert(pid_calls==1&&fabsf(pid_dt-.02f)<1e-7f&&motorSpeed==123);
 reset();start_status=HAL_OK;interval=0;if(setjmp(exit_task)==0)Encoder_Task(NULL);
 assert(!g_encoder_latest.valid&&g_encoder_task_status.zero_interval_samples==1&&pid_calls==0&&motorSpeed==0);
 puts("Encoder startup failure, measured PID interval and zero-dt rejection PASS");
}
