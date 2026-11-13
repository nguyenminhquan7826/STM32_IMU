#ifndef IMU_CONFIG_H
#define IMU_CONFIG_H
#include "imu.h"
#ifdef __cplusplus
extern "C" {
#endif
/* Readback only, not a configuration command. valid means all reads succeeded,
 * not that axes/units are correct for the physical vehicle. */
typedef struct {
  uint8_t chip_id, page_id, unit_sel, operation_mode, power_mode;
  uint8_t axis_map_config, axis_map_sign, i2c_address_7bit;
  uint8_t valid, expected_units_and_ndof;
  HAL_StatusTypeDef last_status;
  uint32_t timestamp_ms;
} BNO055_ConfigSnapshot_t;
extern volatile BNO055_ConfigSnapshot_t g_imu_config_latest;
HAL_StatusTypeDef BNO055_ReadConfig(BNO055_HandleTypeDef *device,
                                    BNO055_ConfigSnapshot_t *output);
#ifdef __cplusplus
}
#endif
#endif
