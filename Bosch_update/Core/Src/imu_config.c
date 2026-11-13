#include "imu_config.h"
#include <stddef.h>
#include <string.h>
volatile BNO055_ConfigSnapshot_t g_imu_config_latest;
HAL_StatusTypeDef BNO055_ReadConfig(BNO055_HandleTypeDef *device,
                                    BNO055_ConfigSnapshot_t *output) {
  BNO055_ConfigSnapshot_t sample;
  uint8_t block[8];
  HAL_StatusTypeDef status = HAL_ERROR;
  if (output == NULL)
    return HAL_ERROR;
  memset(&sample, 0, sizeof(sample));
  sample.last_status = HAL_ERROR;
  if (device == NULL || device->i2c == NULL || device->initialized == 0U) {
    *output = sample;
    return HAL_ERROR;
  }
  sample.i2c_address_7bit = (uint8_t)(device->address >> 1U);
  /* Never switch pages or modes: this function must not alter the sensor. */
  status = HAL_I2C_Mem_Read(device->i2c, device->address, 0x07U,
                            I2C_MEMADD_SIZE_8BIT, &sample.page_id, 1U,
                            BNO055_I2C_TIMEOUT_MS);
  if (status != HAL_OK)
    goto done;
  if (sample.page_id != 0U) {
    status = HAL_ERROR;
    goto done;
  }
  status = HAL_I2C_Mem_Read(device->i2c, device->address, 0x00U,
                            I2C_MEMADD_SIZE_8BIT, &sample.chip_id, 1U,
                            BNO055_I2C_TIMEOUT_MS);
  if (status != HAL_OK)
    goto done;
  status = HAL_I2C_Mem_Read(device->i2c, device->address, 0x3BU,
                            I2C_MEMADD_SIZE_8BIT, block, sizeof(block),
                            BNO055_I2C_TIMEOUT_MS);
  if (status != HAL_OK)
    goto done;
  sample.unit_sel = block[0];
  sample.operation_mode = block[2] & 0x0FU;
  sample.power_mode = block[3] & 0x03U;
  sample.axis_map_config = block[6];
  sample.axis_map_sign = block[7];
  sample.expected_units_and_ndof =
      (uint8_t)(sample.chip_id == BNO055_CHIP_ID_EXPECTED &&
                sample.unit_sel == 0U &&
                sample.operation_mode == (uint8_t)BNO055_MODE_NDOF);
  sample.valid = 1U;
done:
  sample.last_status = status;
  sample.timestamp_ms = HAL_GetTick();
  *output = sample;
  return status;
}
