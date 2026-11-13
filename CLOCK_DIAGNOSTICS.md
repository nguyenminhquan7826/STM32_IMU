# Clock readback for the ~2% STM32/Pi time-rate mismatch

Branch: sensor-config-diagnostics. The code only reads clock/timer state; it does
not switch HSI/HSE, trim the oscillator, change PLL/TIM4, scale HAL timestamps,
or alter FreeRTOS/PID/gyro parameters. Existing IMU bus recovery is retained.
TEL format is unchanged, and no extra UART packet is sent.

Two independent 120s trials reported source seconds per host second of
1.020673210227752 and 1.0201971297374661. This readback helps distinguish a
runtime divider/timer configuration error from a possible reference-frequency
error. It does not independently measure oscillator frequency.

## Load and inspect

1. Update the cloned repo's sensor-config-diagnostics branch.
2. Refresh project Bosch_update, Clean and Build in CubeIDE.
3. Confirm Core/Src/clock_diag.c is compiled; flash the new ELF.
4. Resume for a few seconds, then Suspend and expand `g_clock_config_latest`.

The capture runs once in main's USER CODE section after peripheral initialization
and before creating tasks. Fields describe that capture; they are not a live
frequency meter. Read valid=1 before interpreting fields. GPIO bus recovery does
not change the SYSCLK/PLL/TIM4 configuration captured here.

| Field | Expected from current source |
|---|---|
| valid | 1 |
| sysclk_source_code | 2 = PLL; 0 = HSI, 1 = HSE |
| pll_source_hse | 0 = HSI input; 1 = HSE input |
| pllm / plln / pllp / pllq | 8 / 84 / 2 / 4 |
| hsi_trim | 16 (current default); this is trim code, not measured frequency |
| hsi_factory_cal | Chip-specific factory code; no fixed expected number |
| system_core_clock_hz_nominal | 84000000 |
| sysclk_hz_nominal / hclk_hz_nominal | 84000000 / 84000000 |
| pclk1_hz_nominal / pclk2_hz_nominal | 42000000 / 84000000 |
| ahb_divider / apb1_divider | 1 / 2 |
| tim4_input_hz_nominal | 84000000 |
| tim4_psc / tim4_arr | 83 / 999 |
| tim4_enabled / tim4_update_irq_enabled | 1 / 1 |
| tim4_internal_clock | 1 |
| hal_tick_increment_ms | 1 |
| tim4_update_hz_nominal | 1000 |
| hal_ms_per_nominal_second | 1000 |
| tim4_rate_config_valid / expected_1ms_config | 1 / 1 |

Raw RCC CR/CFGR/PLLCFGR/DCKCFGR and TIM4 CR1/DIER/SMCR are retained. TIMPRE is
included: with TIMPRE=0, timer clock is PCLK1 or 2*PCLK1; with TIMPRE=1 it is
HCLK for APB dividers1/2/4, otherwise 4*PCLK1. A center-aligned/external/slave
counter or disabled timer/update interrupt invalidates the rate configuration
flag. The flag does NOT verify NVIC servicing or actual oscillator accuracy.

All Hz and tick calculations are nominal, using HAL's assumed HSI_VALUE and
HSE_VALUE. HAL reporting 84MHz or expected_1ms_config=1 does not disprove a 2%
real time-rate mismatch. HSI trim/factory values cannot be converted to a precise
physical frequency from this snapshot alone.

## HSE caution

Current project assumes HSE_VALUE=25000000 if HSE were used. This is a software
constant, NOT evidence that the NUCLEO board has a 25MHz oscillator/crystal.
Likewise hse_ready=0 under current HSI operation does not establish that no
external clock is available. Verify board revision, solder bridges and ST-LINK
MCO/crystal source before configuring HSE or bypass and its actual frequency.
Do not copy a generic HSE setup: a wrong source or frequency can prevent boot
and affect UART, motor PWM and scheduler timing.

## Host verification

```bash
gcc -std=c11 -Wall -Wextra -Werror -Itests/stubs -IBosch_update/Core/Inc \
  tests/test_clock_diag.c Bosch_update/Core/Src/clock_diag.c -o /tmp/test_clock_diag
/tmp/test_clock_diag
```

Tests cover runtime register decoding, default 84MHz PLL/APB/tick, alternate
APB1/TIMPRE cases, invalid timer modes/interrupt configuration and arithmetic
boundary conditions. Tests are fake-HAL register models, not physical frequency
measurements. The module was syntax-checked against actual project HAL/CMSIS
headers with host gcc (CMSIS host pointer-width warnings), and existing IMU
startup/readback/recovery host tests pass. No ARM cross-build or board test was
performed here; CubeIDE Build and on-board readback remain required.

## Next decision

Send the complete snapshot first. If the nominal timer rate is wrong, fix the
specific configuration. If dividers and nominal rate are correct, verify the
physical clock source against an independent reference; prefer a verified
stable external source where available. Do not silently hard-code the measured
~1.0204 ratio into all control timing or change gyro scale at this stage.
