#ifndef CLOCK_DIAG_H
#define CLOCK_DIAG_H
#include "stm32f4xx_hal.h"
#ifdef __cplusplus
extern "C" {
#endif
/* Registers are runtime readback. All Hz values remain NOMINAL calculations,
 * not measurements of HSI/HSE accuracy against an independent clock. */
typedef struct {
  uint8_t valid, sysclk_source_code, pll_source_hse, hse_ready,
      hse_bypass_configured;
  uint8_t hsi_trim, hsi_factory_cal, timpre, tim4_enabled,
      tim4_update_irq_enabled;
  uint8_t tim4_internal_clock, tim4_rate_config_valid, expected_1ms_config;
  uint32_t timestamp_ms, rcc_cr, rcc_cfgr, rcc_pllcfgr, rcc_dckcfgr;
  uint32_t pllm, plln, pllp, pllq, ahb_divider, apb1_divider;
  uint32_t system_core_clock_hz_nominal, sysclk_hz_nominal, hclk_hz_nominal;
  uint32_t pclk1_hz_nominal, pclk2_hz_nominal, hsi_hz_assumed, hse_hz_assumed;
  uint32_t tim4_input_hz_nominal, tim4_psc, tim4_arr, tim4_cr1, tim4_dier,
      tim4_smcr;
  uint32_t hal_tick_increment_ms;
  double tim4_update_hz_nominal, hal_ms_per_nominal_second;
} ClockDiagSnapshot_t;
extern volatile ClockDiagSnapshot_t g_clock_config_latest;
void ClockDiag_Capture(void);
#ifdef __cplusplus
}
#endif
#endif
