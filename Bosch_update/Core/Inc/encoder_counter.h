#ifndef ENCODER_COUNTER_H
#define ENCODER_COUNTER_H
#include <stdint.h>
typedef struct {uint16_t previous_counter;uint32_t previous_ms,total_bits;} EncoderCounterState;
typedef struct {int16_t delta_ticks;int32_t total_ticks;uint32_t interval_ms;float speed_mps;} EncoderCounterSample;
void EncoderCounter_Reset(EncoderCounterState*,uint16_t,uint32_t);
/* Counter change must be <32768 counts per interval. Source ms remain
 * uncalibrated: measured dt handles scheduling jitter, not clock-rate error. */
int EncoderCounter_Update(EncoderCounterState*,uint16_t,uint32_t,float,EncoderCounterSample*);
#endif
