#ifndef IMU_BUS_RECOVERY_H
#define IMU_BUS_RECOVERY_H
#include "stm32f4xx_hal.h"
#ifdef __cplusplus
extern "C" {
#endif
typedef struct {
  uint32_t attempts, successes, failures, skipped, timestamp_ms;
  uint8_t in_progress, pulses, scl_before, sda_before, scl_after, sda_after;
  HAL_StatusTypeDef trigger_status, last_status;
} IMU_BusRecovery_t;
extern volatile IMU_BusRecovery_t g_imu_bus_recovery;
/* Single-master I2C1/PB8/PB9 only, IMU task context after a failed HAL call.
 * HAL_OK means bus release/reinit succeeded (or no recovery was needed),
 * NOT that BNO055 was detected. Never call during an active transfer. */
HAL_StatusTypeDef IMU_TryRecoverBus(I2C_HandleTypeDef *i2c,
                                    HAL_StatusTypeDef trigger_status);
#ifdef __cplusplus
}
#endif
#endif
