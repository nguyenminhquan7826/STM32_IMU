#include "telemetry_format.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
int main(int argc,char **argv){
 TelemetryRecord r={0};char out[384],small[12];size_t n;
 (void)argc;(void)argv;
 r.sequence=9;r.tx_ms=100;r.enc_sequence=10;r.enc_ms=99;r.total_ticks=-7;r.delta_ticks=-1;r.speed=-.0125f;r.servo=75;r.pwm=0;
 r.imu_sequence=11;r.imu_ms=96;r.gyro_z_dps=10.0625f;r.yaw_deg=359.9375f;r.calibration=0x34;r.flags=3;
 r.gyro_x_dps=-.125f;r.gyro_y_dps=.25f;r.roll_deg=2.0625f;r.pitch_deg=-3.125f;
 r.read_start_ms=91;r.read_end_ms=96;r.enc_interval_ms=10;r.system_status=5;
 n=Telemetry_Format(out,sizeof(out),&r,1);assert(n>0&&strlen(out)==n);fputs(out,stdout);
 r.sequence++;r.tx_ms+=40;r.enc_sequence++;r.enc_ms+=40;r.imu_sequence++;r.imu_ms+=40;r.read_start_ms+=40;r.read_end_ms+=40;r.total_ticks=17;r.yaw_deg=359.5375f;
 n=Telemetry_Format(out,sizeof(out),&r,2);assert(n>0&&strlen(out)==n);fputs(out,stdout);
 assert(Telemetry_Format(small,sizeof(small),&r,2)==0&&small[0]==0);
 assert(Telemetry_Format(out,sizeof(out),&r,3)==0);
 r.sequence=r.tx_ms=r.enc_sequence=r.enc_ms=r.imu_sequence=r.imu_ms=r.read_start_ms=r.read_end_ms=r.enc_interval_ms=0xffffffffU;
 r.total_ticks=(-2147483647-1);r.delta_ticks=-32768;r.pwm=-2147483647;r.servo=-2147483647;
 r.speed=-9999;r.ax=r.ay=r.gyro_z_dps=r.gyro_x_dps=r.gyro_y_dps=-2000;r.yaw_deg=r.roll_deg=r.pitch_deg=-360;
 n=Telemetry_Format(out,sizeof(out),&r,2);assert(n>0&&n<384);
 fprintf(stderr,"TEL v1/v2 formatting and worst-case bounded output PASS (%zu bytes)\n",n);
}
