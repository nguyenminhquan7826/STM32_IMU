#include "communication.h"

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "task.h"

#include "encoder.h"
#include "imu.h"
#include "telemetry_format.h"

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
#define TX_MESSAGE_SIZE                384U

#ifndef TELEMETRY_PROTOCOL_VERSION
#define TELEMETRY_PROTOCOL_VERSION     2U
#endif
#if TELEMETRY_PROTOCOL_VERSION == 2U
#define TELEMETRY_PERIOD_MS            40U /* Diagnostic payload: preserve UART headroom. */
#elif TELEMETRY_PROTOCOL_VERSION == 1U
#define TELEMETRY_PERIOD_MS            20U
#else
#error Unsupported telemetry version
#endif
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
volatile CommandGuard g_command_guard = {.servo = COMMAND_SERVO_CENTER};
static volatile uint8_t rx_overflow;

static void RingBuffer_Put(RingBuffer_t *ring, uint8_t value)
{
    const uint16_t next =
        (uint16_t)((ring->head + 1U) % RING_BUFFER_SIZE);

    if (next != ring->tail)
    {
        ring->buffer[ring->head] = value;
        ring->head = next;
    } else { rx_overflow = 1U; }
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

void Control_GetCommand(CommandGuard *output)
{
    taskENTER_CRITICAL();
    *output = g_command_guard;
    CommandGuard_Tick(output, HAL_GetTick());
    g_command_guard = *output;
    if (!output->armed || output->speed <= 0.0f) motorSpeed = 0;
    taskEXIT_CRITICAL();
}
void Control_Stop(uint8_t reason)
{
    CommandGuard snapshot;
    taskENTER_CRITICAL();
    snapshot = g_command_guard;
    CommandGuard_Stop(&snapshot, reason);
    g_command_guard = snapshot;
    if (!snapshot.armed || snapshot.speed <= 0.0f) motorSpeed = 0;
    taskEXIT_CRITICAL();
}
void Parse_Command(char *rx_buffer)
{
    CommandGuard snapshot;
    uint32_t seq, now = HAL_GetTick();
    int result;
    char payload[96], reply[112];
    if (!rx_buffer || strncmp(rx_buffer, "CTL,", 4)) return;
    taskENTER_CRITICAL();
    snapshot = g_command_guard;
    result = CommandGuard_Process(&snapshot, rx_buffer, now, &seq);
    g_command_guard = snapshot;
    if (!snapshot.armed || snapshot.speed <= 0.0f) motorSpeed = 0;
    taskEXIT_CRITICAL();
    /* ACK means accepted setpoint, not measured physical actuation. */
    snprintf(payload, sizeof(payload), "ACK,1,%lu,%u,%d,%lu,%lu",
             (unsigned long)seq, snapshot.armed, result,
             (unsigned long)now, (unsigned long)snapshot.sequence);
    snprintf(reply, sizeof(reply), "%s,%04X\r\n", payload,
             Command_CRC(payload, strlen(payload)));
    UART_Send_IT(reply);
}


static void Send_VehicleTelemetry(void)
{
    Encoder_Data_t encoder = {0};
    BNO055_Data_t imu = {0};
    TelemetryRecord r = {0};
    char message[TX_MESSAGE_SIZE];
    if (Encoder_GetLatest(&encoder, TELEMETRY_ENCODER_MAX_AGE_MS)) r.flags |= TELEMETRY_FLAG_ENCODER_VALID;
    if (IMU_GetLatest(&imu, TELEMETRY_IMU_MAX_AGE_MS)) r.flags |= TELEMETRY_FLAG_IMU_VALID;
    r.sequence = ++telemetry_sequence; r.tx_ms = HAL_GetTick();
    r.enc_sequence = encoder.sequence; r.enc_ms = encoder.timestamp_ms;
    r.total_ticks = encoder.total_ticks; r.delta_ticks = encoder.delta_ticks;
    r.speed = encoder.speed_mps; r.servo = steerAngle; r.pwm = motorSpeed;
    r.imu_sequence = imu.sequence; r.imu_ms = imu.timestamp_ms;
    r.ax = imu.linear_accel_x_mps2; r.ay = imu.linear_accel_y_mps2;
    r.gyro_z_dps = imu.gyro_z_dps; r.yaw_deg = imu.yaw_deg;
    r.calibration = imu.calibration_raw;
    r.gyro_x_dps = imu.gyro_x_dps; r.gyro_y_dps = imu.gyro_y_dps;
    r.roll_deg = imu.roll_deg; r.pitch_deg = imu.pitch_deg;
    r.read_start_ms = imu.read_start_ms; r.read_end_ms = imu.read_end_ms;
    r.enc_interval_ms = encoder.interval_ms;
    r.system_status = imu.system_status; r.system_error = imu.system_error;
    if (Telemetry_Format(message, sizeof(message), &r, TELEMETRY_PROTOCOL_VERSION) == 0U) {
        g_uart_tx_truncate_count++; return;
    }
    UART_Send_IT(message);
}

void Communication_Task(void const *argument)
{
    static char command_buffer[RX_BUFFER_SIZE];
    static uint16_t command_index;
    uint32_t last_telemetry_ms;
    uint8_t value;
    uint8_t discard_line = 0U;
    CommandGuard initial;

    (void)argument;

    CommandGuard_Init(&initial);
    g_command_guard = initial;
    rx_overflow = 0U;
    rx_fifo.head = 0U;
    rx_fifo.tail = 0U;
    command_index = 0U;
    last_telemetry_ms = HAL_GetTick();

    /* USART2 RX must be armed in one place only. */
    (void)HAL_UART_Receive_IT(&huart2, &rx_byte, 1U);

    for (;;)
    {
        if (rx_overflow) {
            taskENTER_CRITICAL();
            rx_fifo.tail = rx_fifo.head;
            rx_overflow = 0U;
            taskEXIT_CRITICAL();
            command_index = 0U;
            discard_line = 1U;
            Control_Stop(COMMAND_PARSE);
        }
        while (RingBuffer_Get(&rx_fifo, &value) != 0)
        {
            if (value == (uint8_t)'\n')
            {
                command_buffer[command_index] = '\0';
                if (!discard_line) Parse_Command(command_buffer);
                discard_line = 0U;
                command_index = 0U;
            }
            else if (!discard_line && value != (uint8_t)'\r')
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
                    discard_line = 1U;
                    Control_Stop(COMMAND_PARSE);
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
