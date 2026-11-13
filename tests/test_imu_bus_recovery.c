#include "imu_bus_recovery.h"
#include "task.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
I2C_TypeDef test_i2c1;
GPIO_TypeDef test_gpiob;
static I2C_HandleTypeDef bus;
static int scl_release, sda_release, release_after, edges, stuck_scl,
    permanent_sda;
static int init_calls, deinit_calls, reset_calls, init_fail, deinit_fail;
static uint32_t now;
uint32_t HAL_GetTick(void) { return now; }
void HAL_Delay(uint32_t ms) { now += ms; }
int32_t osDelay(uint32_t ms) {
  now += ms;
  return 0;
}
int xTaskGetSchedulerState(void) { return taskSCHEDULER_RUNNING; }
static void levels(void) {
  test_gpiob.IDR = 0;
  if (scl_release && !stuck_scl)
    test_gpiob.IDR |= GPIO_PIN_8;
  if (sda_release && !permanent_sda && edges >= release_after)
    test_gpiob.IDR |= GPIO_PIN_9;
}
void HAL_GPIO_Init(GPIO_TypeDef *p, GPIO_InitTypeDef *c) {
  (void)p;
  assert(c->Mode == GPIO_MODE_OUTPUT_OD);
  levels();
}
void HAL_GPIO_WritePin(GPIO_TypeDef *p, uint16_t pins, GPIO_PinState state) {
  (void)p;
  if (pins & GPIO_PIN_8) {
    if (state == GPIO_PIN_SET && !scl_release)
      edges++;
    scl_release = state;
  }
  if (pins & GPIO_PIN_9)
    sda_release = state;
  levels();
}
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *p, uint16_t pin) {
  return (p->IDR & pin) ? GPIO_PIN_SET : GPIO_PIN_RESET;
}
HAL_StatusTypeDef HAL_I2C_DeInit(I2C_HandleTypeDef *i) {
  deinit_calls++;
  if (deinit_fail)
    return HAL_ERROR;
  i->State = 0;
  return HAL_OK;
}
HAL_StatusTypeDef HAL_I2C_Init(I2C_HandleTypeDef *i) {
  init_calls++;
  if (init_fail)
    return HAL_ERROR;
  i->State = HAL_I2C_STATE_READY;
  return HAL_OK;
}
void test_i2c_force_reset(void) {
  reset_calls++;
  test_i2c1.SR2 = 0;
}
void test_i2c_release_reset(void) {}
static void reset(void) {
  memset((void *)&g_imu_bus_recovery, 0, sizeof(g_imu_bus_recovery));
  memset(&bus, 0, sizeof(bus));
  bus.Instance = I2C1;
  bus.State = HAL_I2C_STATE_READY;
  scl_release = sda_release = 1;
  release_after = edges = stuck_scl = permanent_sda = 0;
  init_calls = deinit_calls = reset_calls = init_fail = deinit_fail = 0;
  now = 0;
  test_i2c1.SR2 = 2;
  levels();
}
int main(void) {
  reset();
  release_after = 3;
  levels();
  assert(IMU_TryRecoverBus(&bus, HAL_BUSY) == HAL_OK);
  assert(g_imu_bus_recovery.pulses == 3 && g_imu_bus_recovery.successes == 1);
  assert(g_imu_bus_recovery.sda_before == 0 &&
         g_imu_bus_recovery.sda_after == 1);
  assert(init_calls == 1 && deinit_calls == 1 && reset_calls == 1 &&
         edges <= 10);
  reset();
  permanent_sda = 1;
  levels();
  assert(IMU_TryRecoverBus(&bus, HAL_BUSY) == HAL_TIMEOUT);
  assert(g_imu_bus_recovery.pulses == 9 && init_calls == 1 &&
         g_imu_bus_recovery.failures == 1 && now < 100);
  reset();
  stuck_scl = 1;
  levels();
  assert(IMU_TryRecoverBus(&bus, HAL_BUSY) == HAL_TIMEOUT);
  assert(g_imu_bus_recovery.pulses == 0 && init_calls == 1 && now < 100);
  reset();
  assert(IMU_TryRecoverBus(&bus, HAL_BUSY) == HAL_OK);
  assert(g_imu_bus_recovery.pulses == 0 && reset_calls == 1);
  reset();
  test_i2c1.SR2 = 0;
  assert(IMU_TryRecoverBus(&bus, HAL_ERROR) == HAL_OK);
  assert(deinit_calls == 0 && g_imu_bus_recovery.attempts == 0);
  reset();
  bus.Lock = 1;
  assert(IMU_TryRecoverBus(&bus, HAL_BUSY) == HAL_BUSY);
  assert(deinit_calls == 0);
  reset();
  bus.State = 0x21;
  assert(IMU_TryRecoverBus(&bus, HAL_BUSY) == HAL_BUSY);
  assert(deinit_calls == 0);
  reset();
  init_fail = 1;
  assert(IMU_TryRecoverBus(&bus, HAL_BUSY) == HAL_ERROR);
  assert(g_imu_bus_recovery.failures == 1);
  reset();
  deinit_fail = 1;
  assert(IMU_TryRecoverBus(&bus, HAL_BUSY) == HAL_ERROR);
  assert(init_calls == 0);
  reset();
  assert(IMU_TryRecoverBus(&bus, HAL_OK) == HAL_OK);
  assert(deinit_calls == 0);
  puts("Bounded open-drain bus clear, stuck lines, gating and restore failures "
       "PASS");
}
