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

/* Clock readback HAL/CMSIS model (no clock setters are exposed). */
typedef struct {
  volatile uint32_t CR, PLLCFGR, CFGR, DCKCFGR;
} RCC_TypeDef;
typedef struct {
  volatile uint32_t CR1, DIER, SMCR, PSC, ARR, CNT;
} TIM_TypeDef;
extern RCC_TypeDef test_rcc;
extern TIM_TypeDef test_tim4;
extern uint32_t SystemCoreClock;
#define RCC (&test_rcc)
#define TIM4 (&test_tim4)
#define HSI_VALUE 16000000U
#define HSE_VALUE 25000000U
#define RCC_CFGR_SWS 0xCU
#define RCC_CFGR_SWS_Pos 2U
#define RCC_CFGR_HPRE 0xF0U
#define RCC_CFGR_HPRE_Pos 4U
#define RCC_CFGR_PPRE1 0x1C00U
#define RCC_CFGR_PPRE1_Pos 10U
#define RCC_PLLCFGR_PLLM 0x3FU
#define RCC_PLLCFGR_PLLN 0x7FC0U
#define RCC_PLLCFGR_PLLN_Pos 6U
#define RCC_PLLCFGR_PLLP 0x30000U
#define RCC_PLLCFGR_PLLP_Pos 16U
#define RCC_PLLCFGR_PLLQ 0xF000000U
#define RCC_PLLCFGR_PLLQ_Pos 24U
#define RCC_PLLCFGR_PLLSRC (1U << 22)
#define RCC_CR_HSITRIM 0xF8U
#define RCC_CR_HSITRIM_Pos 3U
#define RCC_CR_HSICAL 0xFF00U
#define RCC_CR_HSICAL_Pos 8U
#define RCC_CR_HSEBYP (1U << 18)
#define RCC_CR_HSERDY (1U << 17)
#define RCC_DCKCFGR_TIMPRE (1U << 24)
#define TIM_CR1_CEN 1U
#define TIM_CR1_CMS (3U << 5)
#define TIM_DIER_UIE 1U
#define TIM_SMCR_SMS 7U
#define TIM_SMCR_ECE (1U << 14)
typedef enum { HAL_TICK_FREQ_1KHZ = 1 } HAL_TickFreqTypeDef;
HAL_TickFreqTypeDef HAL_GetTickFreq(void);
uint32_t HAL_RCC_GetSysClockFreq(void);
uint32_t HAL_RCC_GetHCLKFreq(void);
uint32_t HAL_RCC_GetPCLK1Freq(void);
uint32_t HAL_RCC_GetPCLK2Freq(void);
#endif
