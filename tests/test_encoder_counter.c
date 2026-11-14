#include "encoder_counter.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
int main(void){
 EncoderCounterState s;EncoderCounterSample v;
 EncoderCounter_Reset(&s,65530,100);
 assert(EncoderCounter_Update(&s,4,110,.001f,&v));assert(v.delta_ticks==10&&v.total_ticks==10&&v.interval_ms==10);assert(fabsf(v.speed_mps-1.f)<1e-6f);
 assert(EncoderCounter_Update(&s,14,130,.001f,&v));assert(fabsf(v.speed_mps-.5f)<1e-6f);
 assert(!EncoderCounter_Update(&s,20,130,.001f,&v));
 assert(EncoderCounter_Update(&s,24,140,.001f,&v));assert(v.delta_ticks==10);
 assert(EncoderCounter_Update(&s,14,150,.001f,&v));assert(v.delta_ticks==-10);
 EncoderCounter_Reset(&s,0,0xfffffff8U);assert(EncoderCounter_Update(&s,10,2,.001f,&v));assert(v.interval_ms==10);
 s.total_bits=2147483640U;assert(EncoderCounter_Update(&s,20,12,.001f,&v));assert(v.total_ticks==-2147483646);
 assert(!EncoderCounter_Update(&s,25,22,0,&v));
 puts("Encoder measured interval, zero dt, reverse and unsigned wrap PASS");
}
