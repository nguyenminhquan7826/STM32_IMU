#include "clock_diag.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
RCC_TypeDef test_rcc;
TIM_TypeDef test_tim4;
uint32_t SystemCoreClock = 84000000;
static uint32_t hclk = 84000000, pclk1 = 42000000, pclk2 = 84000000;
uint32_t HAL_RCC_GetSysClockFreq(void) { return 84000000; }
uint32_t HAL_RCC_GetHCLKFreq(void) { return hclk; }
uint32_t HAL_RCC_GetPCLK1Freq(void) { return pclk1; }
uint32_t HAL_RCC_GetPCLK2Freq(void) { return pclk2; }
uint32_t HAL_GetTick(void) { return 12; }
HAL_TickFreqTypeDef HAL_GetTickFreq(void) { return HAL_TICK_FREQ_1KHZ; }
static void setup(void) {
  memset(&test_rcc, 0, sizeof(test_rcc));
  memset(&test_tim4, 0, sizeof(test_tim4));
  hclk = 84000000;
  pclk1 = 42000000;
  pclk2 = 84000000;
  test_rcc.CR = (16U << 3) | (127U << 8) | 3U;
  test_rcc.PLLCFGR = 8U | (84U << 6) | (4U << 24);
  test_rcc.CFGR = (2U << 2) | (4U << 10);
  test_tim4.PSC = 83;
  test_tim4.ARR = 999;
  test_tim4.CR1 = 1;
  test_tim4.DIER = 1;
}
int main(void) {
  setup();
  ClockDiag_Capture();
  assert(g_clock_config_latest.valid &&
         g_clock_config_latest.sysclk_source_code == 2);
  assert(g_clock_config_latest.pll_source_hse == 0 &&
         g_clock_config_latest.pllm == 8 && g_clock_config_latest.plln == 84 &&
         g_clock_config_latest.pllp == 2);
  assert(g_clock_config_latest.hsi_trim == 16 &&
         g_clock_config_latest.hsi_factory_cal == 127);
  assert(g_clock_config_latest.apb1_divider == 2 &&
         g_clock_config_latest.tim4_input_hz_nominal == 84000000);
  assert(g_clock_config_latest.tim4_update_hz_nominal == 1000. &&
         g_clock_config_latest.hal_ms_per_nominal_second == 1000.);
  assert(g_clock_config_latest.tim4_rate_config_valid &&
         g_clock_config_latest.timestamp_ms == 12);
  setup();
  test_rcc.CFGR = (2U << 2);
  pclk1 = 84000000;
  ClockDiag_Capture();
  assert(g_clock_config_latest.tim4_input_hz_nominal == 84000000);
  setup();
  test_rcc.CFGR = (2U << 2) | (5U << 10);
  pclk1 = 21000000;
  ClockDiag_Capture();
  assert(g_clock_config_latest.tim4_input_hz_nominal == 42000000);
  test_rcc.DCKCFGR = RCC_DCKCFGR_TIMPRE;
  ClockDiag_Capture();
  assert(g_clock_config_latest.tim4_input_hz_nominal == 84000000);
  test_rcc.CFGR = (2U << 2) | (6U << 10);
  pclk1 = 10500000;
  ClockDiag_Capture();
  assert(g_clock_config_latest.tim4_input_hz_nominal == 42000000);
  setup();
  test_tim4.SMCR = 1;
  ClockDiag_Capture();
  assert(!g_clock_config_latest.tim4_rate_config_valid);
  setup();
  test_tim4.DIER = 0;
  ClockDiag_Capture();
  assert(!g_clock_config_latest.tim4_rate_config_valid);
  setup();
  test_tim4.ARR = 0xffffffffU;
  ClockDiag_Capture();
  assert(g_clock_config_latest.tim4_update_hz_nominal > 0.);
  setup();
  test_rcc.CFGR = (2U << 2) | (8U << 4) | (4U << 10);
  ClockDiag_Capture();
  assert(g_clock_config_latest.ahb_divider == 2);
  setup();
  test_rcc.PLLCFGR |= RCC_PLLCFGR_PLLSRC;
  test_rcc.CR |= RCC_CR_HSERDY | RCC_CR_HSEBYP;
  ClockDiag_Capture();
  assert(g_clock_config_latest.pll_source_hse &&
         g_clock_config_latest.hse_ready &&
         g_clock_config_latest.hse_bypass_configured);
  puts("Read-only clock/PLL/APB/TIM4 decode and nominal tick calculation PASS");
}
