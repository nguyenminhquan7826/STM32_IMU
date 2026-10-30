#ifndef IMU_H
#define IMU_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* STM32 HAL expects the 7-bit I2C address shifted left by one bit. */
#define BNO055_I2C_ADDRESS_LOW       (0x28U << 1)
#define BNO055_I2C_ADDRESS_HIGH      (0x29U << 1)
#define BNO055_I2C_ADDRESS           BNO055_I2C_ADDRESS_HIGH


#define BNO055_CHIP_ID_EXPECTED      0xA0U
#define BNO055_STARTUP_DELAY_MS      700U
#define BNO055_MODE_TO_CONFIG_MS     20U
#define BNO055_CONFIG_TO_MODE_MS     10U
#define BNO055_I2C_TRIALS            3U
#define BNO055_I2C_READY_TIMEOUT_MS  100U
#define BNO055_I2C_TIMEOUT_MS        25U

typedef enum
{
    BNO055_MODE_CONFIG       = 0x00U,
    BNO055_MODE_ACCONLY      = 0x01U,
    BNO055_MODE_MAGONLY      = 0x02U,
    BNO055_MODE_GYRONLY      = 0x03U,
    BNO055_MODE_ACCMAG       = 0x04U,
    BNO055_MODE_ACCGYRO      = 0x05U,
    BNO055_MODE_MAGGYRO      = 0x06U,
    BNO055_MODE_AMG          = 0x07U,
    BNO055_MODE_IMUPLUS      = 0x08U,
    BNO055_MODE_COMPASS      = 0x09U,
    BNO055_MODE_M4G          = 0x0AU,
    BNO055_MODE_NDOF_FMC_OFF = 0x0BU,
    BNO055_MODE_NDOF         = 0x0CU
} BNO055_OperationMode_t;

typedef struct
{
    float accel_x_mps2;
    float accel_y_mps2;
    float accel_z_mps2;

    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;

    /* BNO055 EUL_DATA_X/Y/Z represent heading, roll and pitch. */
    float yaw_deg;
    float roll_deg;
    float pitch_deg;

    float linear_accel_x_mps2;
    float linear_accel_y_mps2;
    float linear_accel_z_mps2;

    int8_t temperature_c;

    uint8_t calibration_raw;
    uint8_t calibration_system;
    uint8_t calibration_gyro;
    uint8_t calibration_accel;
    uint8_t calibration_mag;

    uint8_t system_status;
    uint8_t system_error;

    /* Filled by the application/task after a successful driver read. */
    uint32_t timestamp_ms;
    uint32_t sequence;
    uint8_t valid;
} BNO055_Data_t;

typedef struct
{
    I2C_HandleTypeDef *i2c;
    uint16_t address;
    uint8_t chip_id;
    uint8_t detected;
    uint8_t initialized;
    BNO055_OperationMode_t operation_mode;
    HAL_StatusTypeDef last_status;
    uint32_t read_error_count;
} BNO055_HandleTypeDef;

#define IMU_TASK_PERIOD_MS              10U
#define IMU_TASK_RETRY_DELAY_MS         1000U
#define IMU_TASK_REINIT_ERROR_THRESHOLD 10U

typedef struct
{
    uint8_t initialized;
    HAL_StatusTypeDef last_status;
    uint32_t successful_reads;
    uint32_t failed_reads;
    uint32_t consecutive_errors;
    uint32_t last_update_ms;
} IMU_TaskStatus_t;

/* Defined once in imu.c. Useful in STM32CubeIDE Live Expressions. */
extern BNO055_HandleTypeDef g_bno055;
extern volatile BNO055_Data_t g_imu_latest;
extern volatile IMU_TaskStatus_t g_imu_task_status;



void BNO055_Attach(BNO055_HandleTypeDef *device,
                   I2C_HandleTypeDef *i2c);


void BNO055_SetAddress(BNO055_HandleTypeDef *device,
                       uint16_t hal_address);


HAL_StatusTypeDef BNO055_Detect(BNO055_HandleTypeDef *device);


HAL_StatusTypeDef BNO055_Init(BNO055_HandleTypeDef *device,
                              BNO055_OperationMode_t mode);


HAL_StatusTypeDef BNO055_SetOperationMode(
    BNO055_HandleTypeDef *device,
    BNO055_OperationMode_t mode);


HAL_StatusTypeDef BNO055_ReadData(BNO055_HandleTypeDef *device,
                                  BNO055_Data_t *data);


uint8_t BNO055_IsFullyCalibrated(const BNO055_Data_t *data);

void IMU_Task_Attach(I2C_HandleTypeDef *i2c);


void IMU_Task(void const *argument);


uint8_t IMU_GetLatest(BNO055_Data_t *output,
                      uint32_t max_age_ms);


void IMU_GetTaskStatus(IMU_TaskStatus_t *output);



#ifdef __cplusplus
}
#endif

#endif /* BNO055_H */
