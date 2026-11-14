#include "imu.h"
#include "imu_bus_recovery.h"
#include "imu_config.h"
#include "task.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
/* Startup unit does not run IMU_Task; recovery algorithm has a separate test.
 */
volatile IMU_BusRecovery_t g_imu_bus_recovery;
HAL_StatusTypeDef IMU_TryRecoverBus(I2C_HandleTypeDef *i2c,
                                    HAL_StatusTypeDef status) {
  (void)i2c;
  (void)status;
  assert(!"Unexpected task recovery in startup unit test");
  return HAL_ERROR;
}
I2C_TypeDef test_i2c1;
GPIO_TypeDef test_gpiob;
static uint8_t regs[256];
static uint32_t tick, hal_wait, task_wait;
static int scheduler = taskSCHEDULER_RUNNING, ready_calls, read_calls,
           write_calls;
static uint16_t first_address;
static HAL_StatusTypeDef probe_status = HAL_OK, read_status = HAL_OK;
static int only_low_address, ignore_mode_write;
static int fail_write_reg = -1;
static unsigned reset_writes;
static uint32_t reset_ready_at, reset_boot_ms = 800;
static int reset_never_ready;
static HAL_StatusTypeDef reset_poll_status;
static int reset_poll_failures;
uint32_t HAL_GetTick(void) { return tick; }
void HAL_Delay(uint32_t ms) {
  hal_wait += ms;
  tick += ms;
}
int32_t osDelay(uint32_t ms) {
  task_wait += ms;
  tick += ms;
  return 0;
}
int xTaskGetSchedulerState(void) { return scheduler; }
TickType_t xTaskGetTickCount(void) { return tick; }
void vTaskDelayUntil(TickType_t *t, TickType_t dt) { *t += dt; }
HAL_StatusTypeDef HAL_I2C_IsDeviceReady(I2C_HandleTypeDef *i, uint16_t address,
                                        uint32_t trials, uint32_t timeout) {
  (void)i;
  (void)trials;
  (void)timeout;
  if (ready_calls++ == 0)
    first_address = address;
  if (only_low_address)
    return address == 0x50 ? HAL_OK : HAL_ERROR;
  return probe_status;
}
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *i, uint16_t addr,
                                   uint16_t reg, uint16_t size, uint8_t *out,
                                   uint16_t len, uint32_t timeout) {
  (void)i;
  (void)addr;
  (void)size;
  (void)timeout;
  read_calls++;
  if (read_status != HAL_OK)
    return read_status;
  if (reg == 0 && reset_writes && reset_poll_failures > 0) {
    reset_poll_failures--; return reset_poll_status;
  }
  if (reg == 0 && reset_writes)
    regs[0] = (!reset_never_ready && tick >= reset_ready_at) ? 0xA0 : 0;
  memcpy(out, regs + reg, len);
  tick++;
  return HAL_OK;
}
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *i, uint16_t addr,
                                    uint16_t reg, uint16_t size, uint8_t *in,
                                    uint16_t len, uint32_t timeout) {
  (void)i;
  (void)addr;
  (void)size;
  (void)timeout;
  write_calls++;
  if (reg == fail_write_reg)
    return HAL_ERROR;
  if (reg == 0x3F && *in == 0x20) {
    assert(regs[0x3D] == 0); /* Must enter CONFIG before sensor reset. */
    reset_writes++;
    reset_ready_at = tick + reset_boot_ms;
    memset(regs, 0, sizeof(regs));
  }
  if (!(ignore_mode_write && reg == 0x3D))
    memcpy(regs + reg, in, len);
  return HAL_OK;
}
static I2C_HandleTypeDef bus;
static BNO055_HandleTypeDef d;
static void reset(void) {
  memset(regs, 0, sizeof(regs));
  regs[0] = 0xA0;
  tick = hal_wait = task_wait = 0;
  ready_calls = read_calls = write_calls = only_low_address =
      ignore_mode_write = 0;
  fail_write_reg = -1;
  reset_writes = 0; reset_never_ready = 0; reset_boot_ms = 800;
  reset_poll_status = HAL_OK; reset_poll_failures = 0;
  probe_status = read_status = HAL_OK;
  memset(&bus, 0, sizeof(bus));
  bus.Instance = I2C1;
  test_i2c1.SR2 = 0;
  test_gpiob.IDR = GPIO_PIN_8 | GPIO_PIN_9;
  IMU_Task_Attach(&bus);
  BNO055_Attach(&d, &bus);
  scheduler = taskSCHEDULER_RUNNING;
}
int main(void) {
  reset();
  assert(BNO055_DetectAvailable(&d) == HAL_OK);
  assert(first_address == 0x52 && ready_calls == 1);
  assert(BNO055_Init(&d, BNO055_MODE_NDOF) == HAL_OK);
  assert(ready_calls == 1);
  assert(reset_writes == 1 && regs[0x3D] == 0x0C);
  assert(hal_wait == 0 && task_wait >= 700);
  reset();
  only_low_address = 1;
  assert(BNO055_DetectAvailable(&d) == HAL_OK);
  assert(d.address == 0x50 && ready_calls == 2);
  assert(task_wait == 700);
  ready_calls = 0;
  assert(BNO055_DetectAvailable(&d) == HAL_OK);
  assert(first_address == 0x50 && ready_calls == 1 && task_wait == 700);
  reset();
  probe_status = HAL_BUSY;
  bus.State = 32;
  bus.Lock = 1;
  bus.ErrorCode = 0x20;
  test_i2c1.SR2 = 2;
  test_gpiob.IDR = GPIO_PIN_9;
  assert(BNO055_DetectAvailable(&d) == HAL_BUSY);
  assert(ready_calls == 1);
  assert(g_imu_i2c_first_fault.valid &&
         g_imu_i2c_first_fault.hal_status == HAL_BUSY);
  assert(g_imu_i2c_first_fault.operation == 1 &&
         g_imu_i2c_first_fault.address_7bit == 0x29);
  assert(g_imu_i2c_first_fault.i2c_error_code == 0x20 &&
         g_imu_i2c_first_fault.busy_flag == 1);
  assert(g_imu_i2c_first_fault.scl_level == 0 &&
         g_imu_i2c_first_fault.sda_level == 1);
  probe_status = HAL_ERROR;
  assert(BNO055_Detect(&d) == HAL_ERROR);
  assert(g_imu_i2c_first_fault.hal_status == HAL_BUSY &&
         g_imu_i2c_last_fault.hal_status == HAL_ERROR);
  probe_status = HAL_OK;
  assert(BNO055_Detect(&d) == HAL_OK);
  assert(g_imu_i2c_first_fault.valid);
  reset();
  scheduler = taskSCHEDULER_NOT_STARTED;
  assert(BNO055_Init(&d, BNO055_MODE_NDOF) == HAL_OK);
  assert(hal_wait >= 700 && task_wait == 0);
  reset();
  assert(BNO055_Init(&d, BNO055_MODE_NDOF) == HAL_OK);
  read_status = HAL_TIMEOUT;
  BNO055_Data_t sample;
  assert(BNO055_ReadData(&d, &sample) == HAL_TIMEOUT);
  assert(g_imu_i2c_last_fault.operation == 2 && g_imu_i2c_last_fault.reg == 8);
  reset();
  probe_status = HAL_TIMEOUT;
  assert(BNO055_DetectAvailable(&d) == HAL_TIMEOUT);
  assert(ready_calls == 1);
  reset();
  probe_status = HAL_ERROR;
  assert(BNO055_DetectAvailable(&d) == HAL_ERROR);
  assert(ready_calls == 2 && d.address == 0x52 && task_wait == 700);
  reset();
  regs[0] = 0;
  assert(BNO055_Detect(&d) == HAL_ERROR);
  assert(g_imu_i2c_first_fault.operation == 4 &&
         g_imu_i2c_first_fault.reg == 0);
  reset();
  fail_write_reg = 0x3B;
  assert(BNO055_Init(&d, BNO055_MODE_NDOF) == HAL_ERROR);
  assert(g_imu_i2c_first_fault.stage == 3 &&
         g_imu_i2c_first_fault.operation == 3 &&
         g_imu_i2c_first_fault.reg == 0x3B);
  reset();
  ignore_mode_write = 1;
  assert(BNO055_Init(&d, BNO055_MODE_NDOF) == HAL_ERROR);
  assert(g_imu_i2c_first_fault.operation == 4 &&
         g_imu_i2c_first_fault.reg == 0x3D);
  reset();assert(BNO055_Init(&d,BNO055_MODE_NDOF)==HAL_OK);
  regs[0x14]=0xf0;regs[0x15]=0xff;regs[0x16]=32;regs[0x18]=48;
  regs[0x1a]=160;regs[0x1c]=64;regs[0x1e]=0xb0;regs[0x1f]=0xff;regs[0x39]=5;
  assert(BNO055_ReadData(&d,&sample)==HAL_OK);
  assert(sample.gyro_x_dps==-1.f&&sample.gyro_y_dps==2.f&&sample.gyro_z_dps==3.f);
  assert(sample.yaw_deg==10.f&&sample.roll_deg==4.f&&sample.pitch_deg==-5.f);
  assert(sample.read_end_ms-sample.read_start_ms==2&&sample.timestamp_ms==sample.read_end_ms);
  assert(sample.valid && sample.fusion_healthy);
  regs[0x3A] = 9;
  assert(BNO055_ReadData(&d, &sample) == HAL_OK);
  assert(sample.valid && !sample.fusion_healthy && sample.system_error == 9);
  regs[0x3A] = 0; regs[0x39] = 1;
  assert(BNO055_ReadData(&d, &sample) == HAL_OK);
  assert(sample.valid && !sample.fusion_healthy);
  reset();
  regs[0x3D] = 0x0C; regs[0x3A] = 9;
  assert(BNO055_Init(&d, BNO055_MODE_NDOF) == HAL_OK);
  assert(regs[0x3A] == 0 && regs[0x3D] == 0x0C && reset_writes == 1);
  regs[0x39] = 5; /* All calibration fields remain zero: not a health gate. */
  assert(BNO055_ReadData(&d, &sample) == HAL_OK);
  assert(sample.valid && sample.fusion_healthy && sample.calibration_mag == 0);
  reset(); reset_poll_status = HAL_BUSY; reset_poll_failures = 1;
  assert(BNO055_Init(&d, BNO055_MODE_NDOF) == HAL_BUSY);
  assert(!d.initialized && reset_writes == 1);
  reset(); reset_poll_status = HAL_ERROR; reset_poll_failures = 2;
  assert(BNO055_Init(&d, BNO055_MODE_NDOF) == HAL_OK);
  assert(reset_poll_failures == 0 && d.initialized);
  reset(); reset_poll_status = HAL_TIMEOUT; reset_poll_failures = 2;
  assert(BNO055_Init(&d, BNO055_MODE_NDOF) == HAL_OK);
  assert(reset_poll_failures == 0 && d.initialized);
  reset(); reset_never_ready = 1;
  assert(BNO055_Init(&d, BNO055_MODE_NDOF) == HAL_TIMEOUT);
  assert(!d.initialized && reset_writes == 1 && tick < 2500);
  reset(); fail_write_reg = 0x3F;
  assert(BNO055_Init(&d, BNO055_MODE_NDOF) == HAL_ERROR);
  assert(!d.initialized && reset_writes == 0);
  puts("Preferred address, single startup wait/probe, scheduler delay and "
       "persistent fault capture PASS");
}
