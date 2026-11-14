#include "encoder_counter.h"
#include <stddef.h>
#include <math.h>
#include <limits.h>
void EncoderCounter_Reset(EncoderCounterState *s,uint16_t counter,uint32_t ms){
 if(s==NULL)return;
 s->previous_counter=counter;s->previous_ms=ms;s->total_bits=0;
}
int EncoderCounter_Update(EncoderCounterState *s,uint16_t counter,uint32_t ms,float metres_per_count,EncoderCounterSample *out){
 uint32_t dt;uint16_t bits;int32_t delta;
 if(s==NULL||out==NULL||!isfinite(metres_per_count)||metres_per_count<=0)return 0;
 dt=ms-s->previous_ms;if(dt==0)return 0;
 bits=(uint16_t)(counter-s->previous_counter);
 delta=bits<=INT16_MAX?(int32_t)bits:(int32_t)bits-65536;
 s->total_bits+=(uint32_t)delta;
 out->total_ticks=s->total_bits<=INT32_MAX?(int32_t)s->total_bits:-1-(int32_t)(UINT32_MAX-s->total_bits);
 out->delta_ticks=(int16_t)delta;out->interval_ms=dt;
 out->speed_mps=(float)delta*metres_per_count/((float)dt*.001f);
 s->previous_counter=counter;s->previous_ms=ms;
 return 1;
}
