#ifndef TEST_TASK_H
#define TEST_TASK_H
#include "FreeRTOS.h"
int xTaskGetSchedulerState(void);
TickType_t xTaskGetTickCount(void);
void vTaskDelayUntil(TickType_t *, TickType_t);
#endif
