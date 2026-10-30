#ifndef ENCODER_H
#define ENCODER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "pid.h"

#include <stdint.h>

/* Encoder/mechanical constants. Verify these on the real vehicle. */
#define M_PI                 3.14159265358979323846f
#define WHEEL_RADIUS_M       (0.065f / 2.0f)
#define GEAR_RATIO           (13.0f / 38.0f)
#define PULSES_PER_REV       (11.0f * 4.0f * 19.0f)

#define SAMPLE_MS            10U
#define SAMPLE_TIME_S        ((float)SAMPLE_MS / 1000.0f)

typedef struct
{
    /*
     * Signed counts since STM32 boot. The sign follows the TIM3 encoder
     * direction, so Pi can recover distance even when one TEL packet is lost.
     */
    int32_t total_ticks;
    int16_t delta_ticks;

    float speed_mps;
    uint32_t timestamp_ms;
    uint32_t sequence;
    uint8_t valid;
} Encoder_Data_t;

/*
 * Kept for compatibility and Live Expressions. Localization code should use
 * Encoder_GetLatest() so counts, speed, timestamp and sequence form one
 * consistent snapshot.
 */
extern volatile float car_speed_mps;
extern volatile Encoder_Data_t g_encoder_latest;

float pulses_to_mps(int pulses);
void Encoder_Task(void const *argument);

/*
 * Returns 1 only when a valid encoder sample no older than max_age_ms exists.
 * A max_age_ms value of 0 disables the age check.
 */
uint8_t Encoder_GetLatest(Encoder_Data_t *output, uint32_t max_age_ms);

#ifdef __cplusplus
}
#endif

#endif /* ENCODER_H */
