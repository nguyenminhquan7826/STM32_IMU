#ifndef TEST_HAL_H
#define TEST_HAL_H
#include <stdint.h>
typedef struct {
  volatile uint32_t SR2;
} I2C_TypeDef;
typedef struct {
  volatile uint32_t IDR;
} GPIO_TypeDef;
extern I2C_TypeDef test_i2c1;
extern GPIO_TypeDef test_gpiob;
#define I2C1 (&test_i2c1)
#define GPIOB (&test_gpiob)
#define GPIO_PIN_8 (1U << 8)
#define GPIO_PIN_9 (1U << 9)
#define I2C_SR2_BUSY (1U << 1)
typedef struct {
  I2C_TypeDef *Instance;
  uint32_t State, Lock, ErrorCode;
} I2C_HandleTypeDef;
typedef enum {
  HAL_OK = 0,
  HAL_ERROR = 1,
  HAL_BUSY = 2,
  HAL_TIMEOUT = 3
} HAL_StatusTypeDef;
#define I2C_MEMADD_SIZE_8BIT 1U
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *, uint16_t, uint16_t,
                                   uint16_t, uint8_t *, uint16_t, uint32_t);
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *, uint16_t, uint16_t,
                                    uint16_t, uint8_t *, uint16_t, uint32_t);
HAL_StatusTypeDef HAL_I2C_IsDeviceReady(I2C_HandleTypeDef *, uint16_t, uint32_t,
                                        uint32_t);
void HAL_Delay(uint32_t);
uint32_t HAL_GetTick(void);
#endif
