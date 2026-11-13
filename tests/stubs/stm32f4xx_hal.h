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
#define HAL_I2C_STATE_READY 0x20U
#define HAL_UNLOCKED 0U
#define GPIO_MODE_OUTPUT_OD 1U
#define GPIO_NOPULL 0U
#define GPIO_SPEED_FREQ_LOW 0U
typedef struct {
  uint32_t Pin, Mode, Pull, Speed, Alternate;
} GPIO_InitTypeDef;
typedef enum { GPIO_PIN_RESET = 0, GPIO_PIN_SET = 1 } GPIO_PinState;
void HAL_GPIO_Init(GPIO_TypeDef *, GPIO_InitTypeDef *);
void HAL_GPIO_WritePin(GPIO_TypeDef *, uint16_t, GPIO_PinState);
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *, uint16_t);
void test_i2c_force_reset(void);
void test_i2c_release_reset(void);
#define __HAL_RCC_I2C1_FORCE_RESET() test_i2c_force_reset()
#define __HAL_RCC_I2C1_RELEASE_RESET() test_i2c_release_reset()
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
HAL_StatusTypeDef HAL_I2C_DeInit(I2C_HandleTypeDef *);
HAL_StatusTypeDef HAL_I2C_Init(I2C_HandleTypeDef *);
void HAL_Delay(uint32_t);
uint32_t HAL_GetTick(void);
#endif
