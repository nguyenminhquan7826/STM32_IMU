#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "pid.h"

#include <stdint.h>

/* Pi -> STM32 command receiver. */
void Communication_Task(void const *argument);
void Parse_Command(char *rx_buffer);

/* Non-blocking USART2 transmitter. Call from task context only. */
void UART_Send_IT(const char *message);


/* UART diagnostics for STM32CubeIDE Live Expressions. */
extern volatile uint32_t g_uart_tx_drop_count;
extern volatile uint32_t g_uart_tx_truncate_count;

#ifdef __cplusplus
}
#endif

#endif /* COMMUNICATION_H */
