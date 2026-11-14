#include "telemetry_format.h"
#include <stdio.h>
static uint16_t crc16(const char *s,size_t n){
 uint16_t crc=0xffffU;
 for(size_t i=0;i<n;i++){
  crc^=(uint16_t)(unsigned char)s[i]<<8;
  for(unsigned b=0;b<8;b++)crc=(uint16_t)((crc&0x8000U)?((uint32_t)crc<<1)^0x1021U:((uint32_t)crc<<1));
 }
 return crc;
}
size_t Telemetry_Format(char *out,size_t cap,const TelemetryRecord *r,unsigned version){
 int n,k;size_t used;
 if(out==NULL||cap==0)return 0;
 out[0]=0;
 if(r==NULL||(version!=1U&&version!=2U))return 0;
 n=snprintf(out,cap,"TEL,%u,%lu,%lu,%lu,%lu,%ld,%d,%.4f,%d,%d,%lu,%lu,%.4f,%.4f,%.4f,%.*f,%02X,%u",
  version,(unsigned long)r->sequence,(unsigned long)r->tx_ms,(unsigned long)r->enc_sequence,(unsigned long)r->enc_ms,
  (long)r->total_ticks,r->delta_ticks,(double)r->speed,r->servo,r->pwm,(unsigned long)r->imu_sequence,(unsigned long)r->imu_ms,
  (double)r->ax,(double)r->ay,(double)r->gyro_z_dps,version==1U?2:4,(double)r->yaw_deg,r->calibration,r->flags);
 if(n<=0||(size_t)n>=cap)goto fail;
 used=(size_t)n;
 if(version==2U){
  k=snprintf(out+used,cap-used,",%.4f,%.4f,%.4f,%.4f,%lu,%lu,%lu,%u,%u",
    (double)r->gyro_x_dps,(double)r->gyro_y_dps,(double)r->roll_deg,(double)r->pitch_deg,
    (unsigned long)r->read_start_ms,(unsigned long)r->read_end_ms,(unsigned long)r->enc_interval_ms,r->system_status,r->system_error);
  if(k<=0||(size_t)k>=cap-used)goto fail;
  used+=(size_t)k;
 }
 k=snprintf(out+used,cap-used,",%04X\r\n",(unsigned)crc16(out,used));
 if(k<=0||(size_t)k>=cap-used)goto fail;
 return used+(size_t)k;
fail:
 out[0]=0;return 0;
}
