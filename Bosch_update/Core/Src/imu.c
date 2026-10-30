#include "imu.h"
#include "cmsis_os.h"
#include <stddef.h>
#include <string.h>

#define BNO055_REG_CHIP_ID             0x00U        // đọc mã nhận dạng chip
#define BNO055_REG_PAGE_ID             0x07U        // chọn trang thanh ghi
#define BNO055_REG_ACC_DATA_X_LSB      0x08U        // điểm bắt đầu dữ liệu
#define BNO055_REG_TEMP                0x34U        // nhiệt đô
#define BNO055_REG_UNIT_SEL            0x3BU        // chọn đơn vị đo
#define BNO055_REG_OPR_MODE            0x3DU        //chọn chế độ hoạt động
#define BNO055_REG_PWR_MODE            0x3EU        // chọn chế dộ nguồn
#define BNO055_REG_TEMP_SOURCE         0x40U        // chọn nguồn đo nhiệt độ

#define BNO055_PAGE_0                  0x00U
#define BNO055_POWER_MODE_NORMAL       0x00U
#define BNO055_UNITS_SI_DEGREES        0x00U
#define BNO055_TEMP_SOURCE_GYROSCOPE   0x01U

#define BNO055_SENSOR_BURST_LENGTH     38U
#define BNO055_STATUS_BURST_LENGTH     7U

#define BNO055_SENSOR_OFFSET_ACCEL     0U
#define BNO055_SENSOR_OFFSET_GYRO      12U
#define BNO055_SENSOR_OFFSET_EULER     18U
#define BNO055_SENSOR_OFFSET_LINEAR    32U

#define BNO055_STATUS_OFFSET_TEMP      0U
#define BNO055_STATUS_OFFSET_CALIB     1U
#define BNO055_STATUS_OFFSET_SYS_STAT  5U
#define BNO055_STATUS_OFFSET_SYS_ERR   6U

#define BNO055_ACCEL_SCALE             100.0f
#define BNO055_GYRO_SCALE              16.0f
#define BNO055_EULER_SCALE             16.0f


BNO055_HandleTypeDef g_bno055;
volatile BNO055_Data_t g_imu_latest;
volatile IMU_TaskStatus_t g_imu_task_status;

// đọc length từ thanh nghi first_register
static HAL_StatusTypeDef BNO055_ReadRegisters(
    BNO055_HandleTypeDef *device,
    uint8_t first_register,
    uint8_t *data,
    uint16_t length)
{
    if ((device == NULL) ||
        (device->i2c == NULL) ||
        (data == NULL) ||
        (length == 0U))
    {
        return HAL_ERROR;
    }

    device->last_status = HAL_I2C_Mem_Read(
        device->i2c,
        device->address,
        first_register,
        I2C_MEMADD_SIZE_8BIT,
        data,
        length,
        BNO055_I2C_TIMEOUT_MS);

    return device->last_status;
}

static HAL_StatusTypeDef BNO055_WriteRegister(
    BNO055_HandleTypeDef *device,
    uint8_t reg,
    uint8_t value)
{
    if ((device == NULL) || (device->i2c == NULL))
    {
        return HAL_ERROR;
    }

    device->last_status = HAL_I2C_Mem_Write(
        device->i2c,
        device->address,
        reg,
        I2C_MEMADD_SIZE_8BIT,
        &value,
        1U,
        BNO055_I2C_TIMEOUT_MS);

    return device->last_status;
}

static int16_t BNO055_DecodeInt16(const uint8_t *bytes)
{
    uint16_t value;

    value = (uint16_t)bytes[0];
    value |= (uint16_t)((uint16_t)bytes[1] << 8U);

    return (int16_t)value;
}

static uint8_t BNO055_ModeIsValid(BNO055_OperationMode_t mode)
{
    return ((uint32_t)mode <= (uint32_t)BNO055_MODE_NDOF) ? 1U : 0U;
}

// gắn các thông số cấu hình vào device
void BNO055_Attach(BNO055_HandleTypeDef *device,
                   I2C_HandleTypeDef *i2c)
{
    if (device == NULL)
    {
        return;
    }

    memset(device, 0, sizeof(*device));
    device->i2c = i2c;
    device->address = BNO055_I2C_ADDRESS;
    device->operation_mode = BNO055_MODE_CONFIG;
    device->last_status = HAL_ERROR;
}

void BNO055_SetAddress(BNO055_HandleTypeDef *device,
                       uint16_t hal_address)
{
    if (device == NULL)
    {
        return;
    }

    device->address = hal_address;
    device->detected = 0U;          // sau khi thay đổi ta cần kiểm tra
    device->initialized = 0U;
}

// hàm kiểm tra
HAL_StatusTypeDef BNO055_Detect(BNO055_HandleTypeDef *device)
{
    uint8_t chip_id = 0U;

    if ((device == NULL) || (device->i2c == NULL))
    {
        return HAL_ERROR;
    }

    device->chip_id = 0U;
    device->detected = 0U;
    device->initialized = 0U;
    device->last_status = HAL_ERROR;

    /* The BNO055 can require about 650 ms after power-on. */
    HAL_Delay(BNO055_STARTUP_DELAY_MS);

    device->last_status = HAL_I2C_IsDeviceReady(
        device->i2c,
        device->address,
        BNO055_I2C_TRIALS,
        BNO055_I2C_READY_TIMEOUT_MS);

    if (device->last_status != HAL_OK)
    {
        return device->last_status;
    }

    device->last_status = BNO055_ReadRegisters(
        device,
        BNO055_REG_CHIP_ID,
        &chip_id,
        1U);

    if (device->last_status != HAL_OK)
    {
        return device->last_status;
    }

    device->chip_id = chip_id;

    if (device->chip_id != BNO055_CHIP_ID_EXPECTED)
    {
        device->last_status = HAL_ERROR;
        return HAL_ERROR;
    }

    device->detected = 1U;
    device->last_status = HAL_OK;
    return HAL_OK;
}

HAL_StatusTypeDef BNO055_SetOperationMode(
    BNO055_HandleTypeDef *device,
    BNO055_OperationMode_t mode)
{
    uint8_t current_mode = 0U;

    if ((device == NULL) ||
        (device->i2c == NULL) ||
        (BNO055_ModeIsValid(mode) == 0U))
    {
        return HAL_ERROR;
    }

    if (BNO055_WriteRegister(device,
                             BNO055_REG_PAGE_ID,
                             BNO055_PAGE_0) != HAL_OK)
    {
        return device->last_status;
    }

    if (BNO055_ReadRegisters(device,
                             BNO055_REG_OPR_MODE,
                             &current_mode,
                             1U) != HAL_OK)
    {
        return device->last_status;
    }

    current_mode &= 0x0FU;

    if (current_mode == (uint8_t)mode)
    {
        device->operation_mode = mode;
        device->last_status = HAL_OK;
        return HAL_OK;
    }

    if (current_mode != (uint8_t)BNO055_MODE_CONFIG)
    {
        if (BNO055_WriteRegister(device,
                                 BNO055_REG_OPR_MODE,
                                 (uint8_t)BNO055_MODE_CONFIG) != HAL_OK)
        {
            return device->last_status;
        }

        /* Datasheet requirement: any operation mode -> CONFIGMODE is 19 ms. */
        HAL_Delay(BNO055_MODE_TO_CONFIG_MS);
    }

    if (mode != BNO055_MODE_CONFIG)
    {
        if (BNO055_WriteRegister(device,
                                 BNO055_REG_OPR_MODE,
                                 (uint8_t)mode) != HAL_OK)
        {
            return device->last_status;
        }

        /* Datasheet requirement: CONFIGMODE -> operation mode is 7 ms. */
        HAL_Delay(BNO055_CONFIG_TO_MODE_MS);
    }

    if (BNO055_ReadRegisters(device,
                             BNO055_REG_OPR_MODE,
                             &current_mode,
                             1U) != HAL_OK)
    {
        return device->last_status;
    }

    current_mode &= 0x0FU;

    if (current_mode != (uint8_t)mode)
    {
        device->last_status = HAL_ERROR;
        return HAL_ERROR;
    }

    device->operation_mode = mode;
    device->last_status = HAL_OK;
    return HAL_OK;
}

HAL_StatusTypeDef BNO055_Init(BNO055_HandleTypeDef *device,
                              BNO055_OperationMode_t mode)
{
    if ((device == NULL) ||
        (device->i2c == NULL) ||
        (BNO055_ModeIsValid(mode) == 0U) ||
        (mode == BNO055_MODE_CONFIG))
    {
        return HAL_ERROR;
    }

    device->initialized = 0U;

    if (BNO055_Detect(device) != HAL_OK)
    {
        return device->last_status;
    }

    // chuyển sang chế độ config để cấu hình các thông số dưới đây
    if (BNO055_SetOperationMode(device, BNO055_MODE_CONFIG) != HAL_OK)
    {
        return device->last_status;
    }

    // chọn đọc trang thanh ghi đầu tiên
    if (BNO055_WriteRegister(device,
                             BNO055_REG_PAGE_ID,
                             BNO055_PAGE_0) != HAL_OK)
    {
        return device->last_status;
    }

    // chọn nguồn bình thường
    if (BNO055_WriteRegister(device,
                             BNO055_REG_PWR_MODE,
                             BNO055_POWER_MODE_NORMAL) != HAL_OK)
    {
        return device->last_status;
    }
    HAL_Delay(10U);

    /*
     * UNIT_SEL = 0:
     * accel/linear accel/gravity = m/s^2
     * gyro = degrees/s
     * Euler = degrees
     * temperature = Celsius
     * orientation convention = Windows.
     */
    // chọn đơn vị cho các thông số
    if (BNO055_WriteRegister(device,
                             BNO055_REG_UNIT_SEL,
                             BNO055_UNITS_SI_DEGREES) != HAL_OK)
    {
        return device->last_status;
    }

    // lấy nguồn nhiệt độ từ gyroscope
    if (BNO055_WriteRegister(device,
                             BNO055_REG_TEMP_SOURCE,
                             BNO055_TEMP_SOURCE_GYROSCOPE) != HAL_OK)
    {
        return device->last_status;
    }

    // chuyển sang chế độ yêu cầu
    if (BNO055_SetOperationMode(device, mode) != HAL_OK)
    {
        return device->last_status;
    }

    device->initialized = 1U;
    device->last_status = HAL_OK;
    return HAL_OK;
}

HAL_StatusTypeDef BNO055_ReadData(BNO055_HandleTypeDef *device,
                                  BNO055_Data_t *data)
{
    uint8_t sensor_bytes[BNO055_SENSOR_BURST_LENGTH];
    uint8_t status_bytes[BNO055_STATUS_BURST_LENGTH];
    uint8_t calibration;
    BNO055_Data_t sample;

    // kiểm tra đầu vào
    if ((device == NULL) ||
        (device->i2c == NULL) ||
        (data == NULL) ||
        (device->initialized == 0U))
    {
        return HAL_ERROR;
    }

    //  đọc khôi dữ liệu từ sensor.
    if (BNO055_ReadRegisters(device,
                             BNO055_REG_ACC_DATA_X_LSB,
                             sensor_bytes,
                             BNO055_SENSOR_BURST_LENGTH) != HAL_OK)
    {
        device->read_error_count++;
        return device->last_status;
    }

    // đọc trạng thái
    if (BNO055_ReadRegisters(device,
                             BNO055_REG_TEMP,
                             status_bytes,
                             BNO055_STATUS_BURST_LENGTH) != HAL_OK)
    {
        device->read_error_count++;
        return device->last_status;
    }

    memset(&sample, 0, sizeof(sample));

    // gia tốc (bao gồm trọng lực)
    sample.accel_x_mps2 =
        (float)BNO055_DecodeInt16(&sensor_bytes[
            BNO055_SENSOR_OFFSET_ACCEL + 0U]) / BNO055_ACCEL_SCALE;
    sample.accel_y_mps2 =
        (float)BNO055_DecodeInt16(&sensor_bytes[
            BNO055_SENSOR_OFFSET_ACCEL + 2U]) / BNO055_ACCEL_SCALE;
    sample.accel_z_mps2 =
        (float)BNO055_DecodeInt16(&sensor_bytes[
            BNO055_SENSOR_OFFSET_ACCEL + 4U]) / BNO055_ACCEL_SCALE;

    // giá trị gyroscope
    sample.gyro_x_dps =
        (float)BNO055_DecodeInt16(&sensor_bytes[
            BNO055_SENSOR_OFFSET_GYRO + 0U]) / BNO055_GYRO_SCALE;
    sample.gyro_y_dps =
        (float)BNO055_DecodeInt16(&sensor_bytes[
            BNO055_SENSOR_OFFSET_GYRO + 2U]) / BNO055_GYRO_SCALE;
    sample.gyro_z_dps =
        (float)BNO055_DecodeInt16(&sensor_bytes[
            BNO055_SENSOR_OFFSET_GYRO + 4U]) / BNO055_GYRO_SCALE;

    // góc yaw: hướng xe quay trên mặt phẳng
    sample.yaw_deg =
        (float)BNO055_DecodeInt16(&sensor_bytes[
            BNO055_SENSOR_OFFSET_EULER + 0U]) / BNO055_EULER_SCALE;
    // góc roll: xe nghiêng trái/ phải.
    sample.roll_deg =
        (float)BNO055_DecodeInt16(&sensor_bytes[
            BNO055_SENSOR_OFFSET_EULER + 2U]) / BNO055_EULER_SCALE;
    //góc pitch: đầu xe chúi xuống hay ngẩng lên
    sample.pitch_deg =
        (float)BNO055_DecodeInt16(&sensor_bytes[
            BNO055_SENSOR_OFFSET_EULER + 4U]) / BNO055_EULER_SCALE;

    // đây là thành phần gia tốc đã loại bỏ trọng lực
    sample.linear_accel_x_mps2 =
        (float)BNO055_DecodeInt16(&sensor_bytes[
            BNO055_SENSOR_OFFSET_LINEAR + 0U]) / BNO055_ACCEL_SCALE;
    sample.linear_accel_y_mps2 =
        (float)BNO055_DecodeInt16(&sensor_bytes[
            BNO055_SENSOR_OFFSET_LINEAR + 2U]) / BNO055_ACCEL_SCALE;
    sample.linear_accel_z_mps2 =
        (float)BNO055_DecodeInt16(&sensor_bytes[
            BNO055_SENSOR_OFFSET_LINEAR + 4U]) / BNO055_ACCEL_SCALE;

    // nhiệt độ
    sample.temperature_c =
        (int8_t)status_bytes[BNO055_STATUS_OFFSET_TEMP];

    // thanh ghi calibration các trường: system, gyroscope, accelerometer, magnetometer
    calibration = status_bytes[BNO055_STATUS_OFFSET_CALIB];
    sample.calibration_raw = calibration;
    sample.calibration_system = (calibration >> 6U) & 0x03U;
    sample.calibration_gyro = (calibration >> 4U) & 0x03U;
    sample.calibration_accel = (calibration >> 2U) & 0x03U;
    sample.calibration_mag = calibration & 0x03U;

    sample.system_status =
        status_bytes[BNO055_STATUS_OFFSET_SYS_STAT];
    sample.system_error =
        status_bytes[BNO055_STATUS_OFFSET_SYS_ERR];
    sample.valid = 1U;

    /*
        Việc tạo biến sample lưu data nhằm đảm bảo dự liệu đc toàn vẹn
        dữ liệu từ data luôn là cxac
    */
    *data = sample;
    device->last_status = HAL_OK;
    return HAL_OK;
}

uint8_t BNO055_IsFullyCalibrated(const BNO055_Data_t *data)
{
    if (data == NULL)
    {
        return 0U;
    }

    return ((data->calibration_system == 3U) &&
            (data->calibration_gyro == 3U) &&
            (data->calibration_accel == 3U) &&
            (data->calibration_mag == 3U)) ? 1U : 0U;
}

static void IMU_SetInitialized(uint8_t initialized,
                               HAL_StatusTypeDef status)
{
    taskENTER_CRITICAL();
    g_imu_task_status.initialized = initialized;
    g_imu_task_status.last_status = status;

    if (initialized == 0U)
    {
        g_imu_latest.valid = 0U;
    }
    taskEXIT_CRITICAL();
}

void IMU_Task_Attach(I2C_HandleTypeDef *i2c)
{
    BNO055_Attach(&g_bno055, i2c);
    memset((void *)&g_imu_latest, 0, sizeof(g_imu_latest));
    memset((void *)&g_imu_task_status, 0, sizeof(g_imu_task_status));
    g_imu_task_status.last_status = HAL_ERROR;
}

void IMU_Task(void const *argument)
{
    I2C_HandleTypeDef *i2c;
    uint32_t sequence = 0U;

    i2c = (I2C_HandleTypeDef *)argument;

    if (i2c != NULL)
    {
        IMU_Task_Attach(i2c);
    }

    for (;;)
    {
        TickType_t last_wake_time;
        HAL_StatusTypeDef init_status;

        if (g_bno055.i2c == NULL)
        {
            IMU_SetInitialized(0U, HAL_ERROR);
            osDelay(IMU_TASK_RETRY_DELAY_MS);
            continue;
        }
        HAL_StatusTypeDef detect_status;

        /* Thử địa chỉ 0x28 */
        BNO055_SetAddress(&g_bno055, BNO055_I2C_ADDRESS_LOW);
        detect_status = BNO055_Detect(&g_bno055);

        if (detect_status != HAL_OK)
        {
            /* Không thấy 0x28, thử 0x29 */
            BNO055_SetAddress(&g_bno055, BNO055_I2C_ADDRESS_HIGH);
            detect_status = BNO055_Detect(&g_bno055);
        }

        if (detect_status != HAL_OK)
        {
            IMU_SetInitialized(0U, detect_status);
            osDelay(IMU_TASK_RETRY_DELAY_MS);
            continue;
        }

        init_status = BNO055_Init(&g_bno055, BNO055_MODE_NDOF);


        if (init_status != HAL_OK)
        {
            taskENTER_CRITICAL();
            g_imu_task_status.initialized = 0U;
            g_imu_task_status.last_status = init_status;
            g_imu_task_status.failed_reads++;
            g_imu_task_status.consecutive_errors++;
            g_imu_latest.valid = 0U;
            taskEXIT_CRITICAL();

            osDelay(IMU_TASK_RETRY_DELAY_MS);
            continue;
        }

        taskENTER_CRITICAL();
        g_imu_task_status.initialized = 1U;
        g_imu_task_status.last_status = HAL_OK;
        g_imu_task_status.consecutive_errors = 0U;
        taskEXIT_CRITICAL();

        last_wake_time = xTaskGetTickCount();

        for (;;)
        {
            BNO055_Data_t sample;
            HAL_StatusTypeDef status;
            uint32_t consecutive_errors;

            memset(&sample, 0, sizeof(sample));
            status = BNO055_ReadData(&g_bno055, &sample);

            if (status == HAL_OK)
            {
                sample.timestamp_ms = HAL_GetTick();
                sample.sequence = ++sequence;
                sample.valid = 1U;

                taskENTER_CRITICAL();
                g_imu_latest = sample;
                g_imu_task_status.last_status = HAL_OK;
                g_imu_task_status.successful_reads++;
                g_imu_task_status.consecutive_errors = 0U;
                g_imu_task_status.last_update_ms = sample.timestamp_ms;
                taskEXIT_CRITICAL();

                consecutive_errors = 0U;
            }
            else
            {
                taskENTER_CRITICAL();
                g_imu_task_status.last_status = status;
                g_imu_task_status.failed_reads++;
                g_imu_task_status.consecutive_errors++;
                consecutive_errors =
                    g_imu_task_status.consecutive_errors;
                taskEXIT_CRITICAL();
            }

            if (consecutive_errors >=
                IMU_TASK_REINIT_ERROR_THRESHOLD)
            {
                IMU_SetInitialized(0U, status);
                break;
            }

            vTaskDelayUntil(
                &last_wake_time,
                pdMS_TO_TICKS(IMU_TASK_PERIOD_MS));
        }

        /*
         * Do not spin when the sensor is disconnected. The outer loop will
         * retry detection and NDOF initialization.
         */
        osDelay(100U);
    }
}

uint8_t IMU_GetLatest(BNO055_Data_t *output,
                      uint32_t max_age_ms)
{
    uint32_t age_ms;

    if (output == NULL)
    {
        return 0U;
    }

    taskENTER_CRITICAL();
    *output = g_imu_latest;
    taskEXIT_CRITICAL();

    if (output->valid == 0U)
    {
        return 0U;
    }

    age_ms = HAL_GetTick() - output->timestamp_ms;

    if ((max_age_ms != 0U) && (age_ms > max_age_ms))
    {
        output->valid = 0U;
        return 0U;
    }

    return 1U;
}

void IMU_GetTaskStatus(IMU_TaskStatus_t *output)
{
    if (output == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    output->initialized = g_imu_task_status.initialized;
    output->last_status = g_imu_task_status.last_status;
    output->successful_reads = g_imu_task_status.successful_reads;
    output->failed_reads = g_imu_task_status.failed_reads;
    output->consecutive_errors =
        g_imu_task_status.consecutive_errors;
    output->last_update_ms = g_imu_task_status.last_update_ms;
    taskEXIT_CRITICAL();
}
