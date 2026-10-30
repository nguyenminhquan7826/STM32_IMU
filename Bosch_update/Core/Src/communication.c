#include "communication.h"

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "task.h"

#include "encoder.h"
#include "imu.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern UART_HandleTypeDef huart2;

extern PID_Handle_t pid;
extern int steerAngle;
extern int motorSpeed;

#define RX_BUFFER_SIZE                 128U
#define RING_BUFFER_SIZE               128U
#define TX_QUEUE_SIZE                  8U
#define TX_MESSAGE_SIZE                224U

#define TELEMETRY_PROTOCOL_VERSION     1U
#define TELEMETRY_PERIOD_MS            20U
#define TELEMETRY_ENCODER_MAX_AGE_MS   100U
#define TELEMETRY_IMU_MAX_AGE_MS       100U

#define TELEMETRY_FLAG_ENCODER_VALID   0x01U
#define TELEMETRY_FLAG_IMU_VALID       0x02U

typedef struct
{
    uint8_t buffer[RING_BUFFER_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
} RingBuffer_t;

static RingBuffer_t rx_fifo;
uint8_t rx_byte;

/*
 * A queue slot is immutable from the moment tx_head advances until the USART2
 * completion callback advances tx_tail.
 */
static char tx_buffer[TX_QUEUE_SIZE][TX_MESSAGE_SIZE];
static uint16_t tx_length[TX_QUEUE_SIZE];
static volatile uint8_t tx_head;
static volatile uint8_t tx_tail;
static volatile uint8_t tx_busy;

static uint32_t telemetry_sequence;

volatile uint32_t g_uart_tx_drop_count;
volatile uint32_t g_uart_tx_truncate_count;

static uint16_t CRC16_CCITT(
    const uint8_t *data,
    size_t length)
{
    uint16_t crc = 0xFFFFU;			// khởi tạo thanh ghi crc rỗng
    size_t i;

    for (i = 0U; i < length; i++)
    {
        uint8_t bit;

        crc ^= (uint16_t)data[i] << 8;

        for (bit = 0U; bit < 8U; bit++)
        {
            if ((crc & 0x8000U) != 0U)
            {
                crc = (uint16_t)((crc << 1) ^ 0x1021U);
            }
            else
            {
                crc <<= 1;
            }
        }
    }

    return crc;
}

static void RingBuffer_Put(RingBuffer_t *ring, uint8_t value)
{
    const uint16_t next =
        (uint16_t)((ring->head + 1U) % RING_BUFFER_SIZE);

    if (next != ring->tail)
    {
        ring->buffer[ring->head] = value;
        ring->head = next;
    }
}

static int RingBuffer_Get(RingBuffer_t *ring, uint8_t *value)
{
    if (ring->head == ring->tail)
    {
        return 0;
    }

    *value = ring->buffer[ring->tail];
    ring->tail =
        (uint16_t)((ring->tail + 1U) % RING_BUFFER_SIZE);
    return 1;
}

static HAL_StatusTypeDef UART_StartTailTransfer(void)
{
    return HAL_UART_Transmit_IT(
        &huart2,
        (uint8_t *)tx_buffer[tx_tail],
        tx_length[tx_tail]);
}

void UART_Send_IT(const char *message)
{
    size_t message_length;
    uint8_t next;

    if (message == NULL)
    {
        return;
    }

    message_length = strlen(message);

    /*
     * UART_Send_IT can be called by more than one task. Protect the producer
     * indices and prevent the TX ISR from changing tx_tail halfway through.
     */
    taskENTER_CRITICAL();

    next = (uint8_t)((tx_head + 1U) % TX_QUEUE_SIZE);

    if (next == tx_tail)
    {
        g_uart_tx_drop_count++;
        taskEXIT_CRITICAL();
        return;
    }

    if (message_length >= TX_MESSAGE_SIZE)
    {
        message_length = TX_MESSAGE_SIZE - 1U;
        g_uart_tx_truncate_count++;
    }

    memcpy(tx_buffer[tx_head], message, message_length);
    tx_buffer[tx_head][message_length] = '\0';
    tx_length[tx_head] = (uint16_t)message_length;
    tx_head = next;

    if (tx_busy == 0U)
    {
        tx_busy = 1U;

        if (UART_StartTailTransfer() != HAL_OK)
        {
            tx_tail =
                (uint8_t)((tx_tail + 1U) % TX_QUEUE_SIZE);
            tx_busy = 0U;
            g_uart_tx_drop_count++;
        }
    }

    taskEXIT_CRITICAL();
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *uart)
{
    if (uart->Instance != USART2)
    {
        return;
    }

    tx_tail = (uint8_t)((tx_tail + 1U) % TX_QUEUE_SIZE);

    if (tx_tail != tx_head)
    {
        if (UART_StartTailTransfer() != HAL_OK)
        {
            tx_tail =
                (uint8_t)((tx_tail + 1U) % TX_QUEUE_SIZE);
            tx_busy = 0U;
            g_uart_tx_drop_count++;
        }
    }
    else
    {
        tx_busy = 0U;
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart)
{
    if (uart->Instance == USART2)
    {
        RingBuffer_Put(&rx_fifo, rx_byte);
        (void)HAL_UART_Receive_IT(&huart2, &rx_byte, 1U);
    }
}

void Parse_Command(char *rx_buffer)
{
    if ((rx_buffer != NULL) &&
        (strncmp(rx_buffer, "CMD", 3U) == 0))
    {
        float speed_mps;
        int angle;

        if (sscanf(
                rx_buffer,
                "CMD,%f,%d",
                &speed_mps,
                &angle) == 2)
        {
            pid.setpoint = speed_mps;
            steerAngle = angle;
        }
    }
}


static void Send_VehicleTelemetry(void)
{
    Encoder_Data_t encoder = {0};
    BNO055_Data_t imu = {0};
    uint8_t flags = 0U;
    int steering_command_deg;
    int pwm_command;
    char message[TX_MESSAGE_SIZE];
    int payload_length;
    int suffix_length;
    uint16_t crc;

    if (Encoder_GetLatest(
            &encoder,
            TELEMETRY_ENCODER_MAX_AGE_MS) != 0U)
    {
        flags |= TELEMETRY_FLAG_ENCODER_VALID;
    }

    if (IMU_GetLatest(
            &imu,
            TELEMETRY_IMU_MAX_AGE_MS) != 0U)
    {
        flags |= TELEMETRY_FLAG_IMU_VALID;
    }

    /*
     * These aligned int reads are atomic on Cortex-M4. The fields are commands:
     * steering_command_deg is the servo command, not a measured road-wheel
     * angle; pwm_command is the current PID output.
     */
    steering_command_deg = steerAngle;
    pwm_command = motorSpeed;
    telemetry_sequence++;

    /*
     * Protocol v1 fields:
     * TEL,version,packet_seq,tx_ms,
     *     enc_seq,enc_ms,enc_total,enc_delta,speed_mps,
     *     steering_cmd_deg,pwm_cmd,
     *     imu_seq,imu_ms,lin_ax,lin_ay,gyro_z_dps,yaw_deg,
     *     calib,flags,crc16
     *
     * Signed enc_total/enc_delta encode direction.
     * flags bit 0: encoder valid/fresh
     * flags bit 1: IMU valid/fresh
     * CRC-16/CCITT-FALSE covers bytes from "TEL" through flags, excluding
     * the comma before crc16 and excluding CR/LF.
     */
    payload_length = snprintf(
        message,
        sizeof(message),
        "TEL,%u,%lu,%lu,"
        "%lu,%lu,%ld,%d,%.4f,%d,%d,"
        "%lu,%lu,%.4f,%.4f,%.4f,%.2f,%02X,%u",
        (unsigned int)TELEMETRY_PROTOCOL_VERSION,
        (unsigned long)telemetry_sequence,
        (unsigned long)HAL_GetTick(),
        (unsigned long)encoder.sequence,
        (unsigned long)encoder.timestamp_ms,
        (signed long)encoder.total_ticks,
        (int)encoder.delta_ticks,
        (double)encoder.speed_mps,
        steering_command_deg,
        pwm_command,
        (unsigned long)imu.sequence,
        (unsigned long)imu.timestamp_ms,
        (double)imu.linear_accel_x_mps2,
        (double)imu.linear_accel_y_mps2,
        (double)imu.gyro_z_dps,
        (double)imu.yaw_deg,
        (unsigned int)imu.calibration_raw,
        (unsigned int)flags);

    if ((payload_length <= 0) ||
        ((uint32_t)payload_length >= (uint32_t)sizeof(message)))
    {
        g_uart_tx_truncate_count++;
        return;
    }

    crc = CRC16_CCITT(
        (const uint8_t *)message,
        (size_t)payload_length);

    suffix_length = snprintf(
        &message[payload_length],
        sizeof(message) - (size_t)payload_length,
        ",%04X\r\n",
        (unsigned int)crc);

    if ((suffix_length > 0) &&
        ((size_t)suffix_length <
         (sizeof(message) - (size_t)payload_length)))
    {
        UART_Send_IT(message);
    }
    else
    {
        g_uart_tx_truncate_count++;
    }
}

void Communication_Task(void const *argument)
{
    static char command_buffer[RX_BUFFER_SIZE];
    static uint16_t command_index;
    uint32_t last_telemetry_ms;
    uint8_t value;

    (void)argument;

    rx_fifo.head = 0U;
    rx_fifo.tail = 0U;
    command_index = 0U;
    last_telemetry_ms = HAL_GetTick();

    /* USART2 RX must be armed in one place only. */
    (void)HAL_UART_Receive_IT(&huart2, &rx_byte, 1U);

    for (;;)
    {
        while (RingBuffer_Get(&rx_fifo, &value) != 0)
        {
            if (value == (uint8_t)'\n')
            {
                command_buffer[command_index] = '\0';
                Parse_Command(command_buffer);
                command_index = 0U;
            }
            else if (value != (uint8_t)'\r')
            {
                if (command_index <
                    (uint16_t)(sizeof(command_buffer) - 1U))
                {
                    command_buffer[command_index] = (char)value;
                    command_index++;
                }
                else
                {
                    command_index = 0U;
                }
            }
        }

        {
            const uint32_t now_ms = HAL_GetTick();

            if ((now_ms - last_telemetry_ms) >=
                TELEMETRY_PERIOD_MS)
            {
                last_telemetry_ms = now_ms;
                Send_VehicleTelemetry();
            }
        }

        osDelay(1U);
    }
}
