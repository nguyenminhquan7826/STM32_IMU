#include "encoder.h"
#include "encoder_counter.h"

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "task.h"

#include <stddef.h>

extern TIM_HandleTypeDef htim3;
extern PID_Handle_t pid;
extern int motorSpeed;

volatile uint8_t flag = 0U;
volatile float car_speed_mps = 0.0f;
volatile Encoder_Data_t g_encoder_latest;
volatile Encoder_TaskStatus_t g_encoder_task_status;

float pulses_to_mps(int pulses)
{
    const float motor_revolutions =
        ((float)pulses) / PULSES_PER_REV;
    const float motor_rps =
        motor_revolutions / SAMPLE_TIME_S;
    const float wheel_rps =
        motor_rps * GEAR_RATIO;

    return wheel_rps * (2.0f * M_PI * WHEEL_RADIUS_M);
}

uint8_t Encoder_GetLatest(
    Encoder_Data_t *output,
    uint32_t max_age_ms)
{
    Encoder_Data_t snapshot;
    uint32_t now_ms;

    if (output == NULL)
    {
        return 0U;
    }

    taskENTER_CRITICAL();
    snapshot.total_ticks = g_encoder_latest.total_ticks;
    snapshot.delta_ticks = g_encoder_latest.delta_ticks;
    snapshot.speed_mps = g_encoder_latest.speed_mps;
    snapshot.timestamp_ms = g_encoder_latest.timestamp_ms;
    snapshot.sequence = g_encoder_latest.sequence;
    snapshot.interval_ms = g_encoder_latest.interval_ms;
    snapshot.valid = g_encoder_latest.valid;
    taskEXIT_CRITICAL();

    *output = snapshot;

    if (snapshot.valid == 0U)
    {
        return 0U;
    }

    if (max_age_ms == 0U)
    {
        return 1U;
    }

    now_ms = HAL_GetTick();
    return ((now_ms - snapshot.timestamp_ms) <= max_age_ms) ? 1U : 0U;
}

void Encoder_Task(void const *argument)
{
    TickType_t last_wake_time;
    EncoderCounterState counter;
    const float metres_per_count = GEAR_RATIO * (2.0f*M_PI*WHEEL_RADIUS_M)/PULSES_PER_REV;
    (void)argument;
    g_encoder_task_status.start_status = HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    if (g_encoder_task_status.start_status != HAL_OK) {
        taskENTER_CRITICAL();
        g_encoder_latest.valid = 0U;
        g_encoder_task_status.initialized = 0U;
        motorSpeed = 0;
        taskEXIT_CRITICAL();
        /* Keep telemetry alive to expose start_status; no PID on invalid feedback. */
        for (;;) { osDelay(100U); }
    }
    EncoderCounter_Reset(&counter, (uint16_t)__HAL_TIM_GET_COUNTER(&htim3), HAL_GetTick());
    g_encoder_task_status.initialized = 1U;
    last_wake_time = xTaskGetTickCount();
    for (;;) {
        EncoderCounterSample sample;
        uint32_t timestamp_ms;
        uint16_t current_counter;
        /* First sample also needs a nonzero measurement window. */
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(SAMPLE_MS));
        current_counter = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
        timestamp_ms = HAL_GetTick();
        if (!EncoderCounter_Update(&counter, current_counter, timestamp_ms, metres_per_count, &sample)) {
            taskENTER_CRITICAL();
            g_encoder_latest.valid = 0U;
            g_encoder_task_status.zero_interval_samples++;
            motorSpeed = 0;
            taskEXIT_CRITICAL();
            continue;
        }
        taskENTER_CRITICAL();
        car_speed_mps = sample.speed_mps;
        g_encoder_latest.total_ticks = sample.total_ticks;
        g_encoder_latest.delta_ticks = sample.delta_ticks;
        g_encoder_latest.speed_mps = sample.speed_mps;
        g_encoder_latest.timestamp_ms = timestamp_ms;
        g_encoder_latest.sequence++;
        g_encoder_latest.interval_ms = sample.interval_ms;
        g_encoder_latest.valid = 1U;
        g_encoder_task_status.last_interval_ms = sample.interval_ms;
        taskEXIT_CRITICAL();
        motorSpeed = (int)PID_Update(&pid, sample.speed_mps, sample.interval_ms*.001f);
        flag = 1U;
    }
}
