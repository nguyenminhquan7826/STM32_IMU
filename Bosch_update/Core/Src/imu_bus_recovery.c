#include "imu_bus_recovery.h"
#include "cmsis_os.h"
#include "task.h"
#include <stddef.h>
volatile IMU_BusRecovery_t g_imu_bus_recovery;
static void delay_ms(uint32_t ms) {
  if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
    (void)osDelay(ms);
  else
    HAL_Delay(ms);
}
static uint8_t level(uint16_t pin) {
  return HAL_GPIO_ReadPin(GPIOB, pin) == GPIO_PIN_SET;
}
static uint8_t wait_scl(void) {
  for (unsigned int n = 0; n < 5U; n++) {
    if (level(GPIO_PIN_8))
      return 1U;
    delay_ms(1U);
  }
  return level(GPIO_PIN_8);
}
HAL_StatusTypeDef IMU_TryRecoverBus(I2C_HandleTypeDef *i2c,
                                    HAL_StatusTypeDef trigger_status) {
  HAL_StatusTypeDef result, restored;
  GPIO_InitTypeDef gpio = {0};
  if (i2c == NULL || i2c->Instance != I2C1)
    return HAL_ERROR;
  if (trigger_status == HAL_OK)
    return HAL_OK;
  /* Refuse to interrupt a HAL transfer or steal its lock. */
  if (i2c->State != HAL_I2C_STATE_READY || i2c->Lock != HAL_UNLOCKED) {
    g_imu_bus_recovery.skipped++;
    return HAL_BUSY;
  }
  if ((i2c->Instance->SR2 & I2C_SR2_BUSY) == 0U && level(GPIO_PIN_8) &&
      level(GPIO_PIN_9)) {
    g_imu_bus_recovery.skipped++;
    return HAL_OK;
  }
  g_imu_bus_recovery.in_progress = 1U;
  g_imu_bus_recovery.attempts++;
  g_imu_bus_recovery.trigger_status = trigger_status;
  g_imu_bus_recovery.pulses = 0U;
  g_imu_bus_recovery.scl_before = level(GPIO_PIN_8);
  g_imu_bus_recovery.sda_before = level(GPIO_PIN_9);
  result = HAL_I2C_DeInit(i2c);
  if (result != HAL_OK)
    goto done;
  /* Preload released outputs; GPIO SET in open-drain means release, never
   * actively driving 3.3V against the sensor. External pullups are required. */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_SET);
  gpio.Pin = GPIO_PIN_8 | GPIO_PIN_9;
  gpio.Mode = GPIO_MODE_OUTPUT_OD;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &gpio);
  if (!wait_scl()) {
    result = HAL_TIMEOUT;
    goto restore;
  }
  while (!level(GPIO_PIN_9) && g_imu_bus_recovery.pulses < 9U) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
    delay_ms(1U);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
    g_imu_bus_recovery.pulses++;
    if (!wait_scl()) {
      result = HAL_TIMEOUT;
      goto restore;
    }
    delay_ms(1U);
  }
  if (!level(GPIO_PIN_9)) {
    result = HAL_TIMEOUT;
    goto restore;
  }
  /* STOP: SDA rises while SCL is high. SCL-low setup avoids a START. */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
  delay_ms(1U);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
  if (!wait_scl()) {
    result = HAL_TIMEOUT;
    goto restore;
  }
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
  delay_ms(1U);
  result = level(GPIO_PIN_9) ? HAL_OK : HAL_TIMEOUT;
restore:
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_SET);
  /* Clear the STM32 peripheral state and restore AF_OD via its MSP Init. */
  __HAL_RCC_I2C1_FORCE_RESET();
  __HAL_RCC_I2C1_RELEASE_RESET();
  restored = HAL_I2C_Init(i2c);
  if (restored != HAL_OK)
    result = restored;
  if (result == HAL_OK && (!level(GPIO_PIN_8) || !level(GPIO_PIN_9) ||
                           (i2c->Instance->SR2 & I2C_SR2_BUSY) != 0U))
    result = HAL_BUSY;
done:
  g_imu_bus_recovery.scl_after = level(GPIO_PIN_8);
  g_imu_bus_recovery.sda_after = level(GPIO_PIN_9);
  g_imu_bus_recovery.last_status = result;
  g_imu_bus_recovery.timestamp_ms = HAL_GetTick();
  if (result == HAL_OK)
    g_imu_bus_recovery.successes++;
  else
    g_imu_bus_recovery.failures++;
  g_imu_bus_recovery.in_progress = 0U;
  return result;
}
