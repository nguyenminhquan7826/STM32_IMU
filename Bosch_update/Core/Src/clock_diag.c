#include "clock_diag.h"
#include <string.h>
volatile ClockDiagSnapshot_t g_clock_config_latest;
void ClockDiag_Capture(void) {
  ClockDiagSnapshot_t s;
  uint32_t ahb_code, apb_code;
  static const uint16_t ahb_dividers[8] = {2, 4, 8, 16, 64, 128, 256, 512};
  memset(&s, 0, sizeof(s));
  s.rcc_cr = RCC->CR;
  s.rcc_cfgr = RCC->CFGR;
  s.rcc_pllcfgr = RCC->PLLCFGR;
  s.rcc_dckcfgr = RCC->DCKCFGR;
  s.sysclk_source_code =
      (uint8_t)((s.rcc_cfgr & RCC_CFGR_SWS) >> RCC_CFGR_SWS_Pos);
  s.pll_source_hse = (s.rcc_pllcfgr & RCC_PLLCFGR_PLLSRC) != 0U;
  s.hse_ready = (s.rcc_cr & RCC_CR_HSERDY) != 0U;
  s.hse_bypass_configured = (s.rcc_cr & RCC_CR_HSEBYP) != 0U;
  s.hsi_trim = (uint8_t)((s.rcc_cr & RCC_CR_HSITRIM) >> RCC_CR_HSITRIM_Pos);
  s.hsi_factory_cal =
      (uint8_t)((s.rcc_cr & RCC_CR_HSICAL) >> RCC_CR_HSICAL_Pos);
  s.pllm = s.rcc_pllcfgr & RCC_PLLCFGR_PLLM;
  s.plln = (s.rcc_pllcfgr & RCC_PLLCFGR_PLLN) >> RCC_PLLCFGR_PLLN_Pos;
  s.pllp =
      2U * (((s.rcc_pllcfgr & RCC_PLLCFGR_PLLP) >> RCC_PLLCFGR_PLLP_Pos) + 1U);
  s.pllq = (s.rcc_pllcfgr & RCC_PLLCFGR_PLLQ) >> RCC_PLLCFGR_PLLQ_Pos;
  ahb_code = (s.rcc_cfgr & RCC_CFGR_HPRE) >> RCC_CFGR_HPRE_Pos;
  apb_code = (s.rcc_cfgr & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos;
  s.ahb_divider = ahb_code < 8U ? 1U : ahb_dividers[ahb_code - 8U];
  s.apb1_divider = apb_code < 4U ? 1U : (1U << (apb_code - 3U));
  s.system_core_clock_hz_nominal = SystemCoreClock;
  s.sysclk_hz_nominal = HAL_RCC_GetSysClockFreq();
  s.hclk_hz_nominal = HAL_RCC_GetHCLKFreq();
  s.pclk1_hz_nominal = HAL_RCC_GetPCLK1Freq();
  s.pclk2_hz_nominal = HAL_RCC_GetPCLK2Freq();
  s.hsi_hz_assumed = HSI_VALUE;
  s.hse_hz_assumed = HSE_VALUE;
  s.timpre = (s.rcc_dckcfgr & RCC_DCKCFGR_TIMPRE) != 0U;
  if (s.timpre == 0U)
    s.tim4_input_hz_nominal =
        s.pclk1_hz_nominal * (s.apb1_divider == 1U ? 1U : 2U);
  else
    s.tim4_input_hz_nominal =
        s.apb1_divider <= 4U ? s.hclk_hz_nominal : 4U * s.pclk1_hz_nominal;
  s.tim4_psc = TIM4->PSC;
  s.tim4_arr = TIM4->ARR;
  s.tim4_cr1 = TIM4->CR1;
  s.tim4_dier = TIM4->DIER;
  s.tim4_smcr = TIM4->SMCR;
  s.tim4_enabled = (s.tim4_cr1 & TIM_CR1_CEN) != 0U;
  s.tim4_update_irq_enabled = (s.tim4_dier & TIM_DIER_UIE) != 0U;
  s.tim4_internal_clock = (s.tim4_smcr & (TIM_SMCR_SMS | TIM_SMCR_ECE)) == 0U;
  s.hal_tick_increment_ms = (uint32_t)HAL_GetTickFreq();
  /* Cast before adding one, avoiding uint32 overflow on a maximum ARR. */
  s.tim4_update_hz_nominal = (double)s.tim4_input_hz_nominal /
                             ((double)s.tim4_psc + 1.) /
                             ((double)s.tim4_arr + 1.);
  s.hal_ms_per_nominal_second =
      s.tim4_update_hz_nominal * s.hal_tick_increment_ms;
  s.tim4_rate_config_valid = s.tim4_enabled && s.tim4_update_irq_enabled &&
                             s.tim4_internal_clock &&
                             (s.tim4_cr1 & TIM_CR1_CMS) == 0U;
  s.expected_1ms_config = s.tim4_rate_config_valid &&
                          s.hal_ms_per_nominal_second >= 999.99 &&
                          s.hal_ms_per_nominal_second <= 1000.01;
  s.timestamp_ms = HAL_GetTick();
  g_clock_config_latest.valid = 0U;
  g_clock_config_latest = s;
  g_clock_config_latest.valid = 1U;
}
