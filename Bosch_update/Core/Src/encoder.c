#include "encoder.h"

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
    uint16_t previous_counter;
    int32_t total_ticks;

    (void)argument;

    (void)HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    previous_counter =
        (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
    total_ticks = 0;
    last_wake_time = xTaskGetTickCount();

    for (;;)
    {
        uint16_t current_counter;
        int16_t pulse_delta;
        float measured_speed_mps;
        float output;
        uint32_t timestamp_ms;

        current_counter =
            (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
        timestamp_ms = HAL_GetTick();

        /*
         * Signed 16-bit subtraction handles one timer wrap correctly as long
         * as fewer than 32768 counts occur during one 10 ms sample.
         */
        pulse_delta = (int16_t)(
            (int32_t)current_counter -
            (int32_t)previous_counter);

        total_ticks += (int32_t)pulse_delta;
        measured_speed_mps = pulses_to_mps((int)pulse_delta);

        taskENTER_CRITICAL();
        car_speed_mps = measured_speed_mps;
        g_encoder_latest.total_ticks = total_ticks;
        g_encoder_latest.delta_ticks = pulse_delta;
        g_encoder_latest.speed_mps = measured_speed_mps;
        g_encoder_latest.timestamp_ms = timestamp_ms;
        g_encoder_latest.sequence++;
        g_encoder_latest.valid = 1U;
        taskEXIT_CRITICAL();

        output = PID_Update(
            &pid,
            measured_speed_mps,
            SAMPLE_TIME_S);
        motorSpeed = (int)output;

        /*
         * Do not call Send_SpeedFeedback() here at 100 Hz. The combined TEL
         * packet is sent by Communication_Task at 50 Hz.
         */

        flag = 1U;
        previous_counter = current_counter;

        vTaskDelayUntil(
            &last_wake_time,
            pdMS_TO_TICKS(SAMPLE_MS));
    }
}
