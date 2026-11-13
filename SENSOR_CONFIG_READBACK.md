# Read-only BNO055 configuration snapshot

Branch: sensor-config-diagnostics. TEL v1, UART commands, motor PID, sensor units
and axis mapping are unchanged. No new packet is sent on UART. This firmware
change requires an STM32CubeIDE rebuild and flash before it can be observed.
It does not change Kalman on Pi.

After every successful NDOF initialization, the IMU task reads configuration
once, outside the normal acquisition loop. Inspect `g_imu_config_latest` in
STM32CubeIDE Expressions while halted or use Live Expressions cautiously:

| Field | Meaning / expected candidate |
|---|---|
| valid | 1 if every read succeeded; does NOT certify physical alignment |
| last_status | HAL_OK=0; errors are visible even if normal IMU reading continues |
| chip_id | 0xA0 |
| i2c_address_7bit | 0x28 or 0x29; HAL address is shifted left |
| page_id | 0; other pages are rejected, never switched by diagnostics |
| unit_sel | 0x00 as written by existing initialization |
| operation_mode | 0x0C (NDOF), low nibble |
| power_mode | 0 (normal), low two bits |
| axis_map_config | Sensor register 0x41; typical default 0x24, not forced here |
| axis_map_sign | Sensor register 0x42; typical default 0x00, not forced here |
| expected_units_and_ndof | chip ID/unit/mode match expected, NOT a calibration/axis flag |
| timestamp_ms | HAL timestamp at completion of diagnostic attempt |

The routine reads PAGE_ID, CHIP_ID and 8 bytes beginning at UNIT_SEL (0x3B).
It never writes registers or switches mode, so it cannot silently repair a bad
configuration. A read error does not disable normal telemetry. The snapshot is
published inside a short critical section, while I2C transfers stay outside it.
The three reads are not a hardware-atomic snapshot; only the IMU task should
access the sensor during this initialization stage. It is not refreshed every
sample and must not be mistaken for proof of the current calibration state.

New sources: Core/Inc/imu_config.h and Core/Src/imu_config.c. They are included
by the existing managed Core source folder. Refresh the project and Clean/Build
in CubeIDE; confirm imu_config.c is present in the compilation log. Do not rely
on committed Debug objects/binaries: they have NOT been rebuilt here.

Host validation only:

```bash
gcc -std=c11 -Wall -Wextra -Werror -Itests/stubs -IBosch_update/Core/Inc \
  tests/test_imu_config.c Bosch_update/Core/Src/imu_config.c -o /tmp/test_imu_config
/tmp/test_imu_config
```

This compiles the actual readback source against a minimal mock HAL, checking
register decoding, wrong unit mode, nonzero page, all three I2C failure points
and invalid pointers. It is not a firmware cross-build or hardware I2C test.
The test stub deliberately provides no write API, enforcing a read-only module.
No arm-none-eabi-gcc toolchain is installed in the current analysis environment.
An attempted syntax check of the complete imu.c with the real HAL/FreeRTOS
headers was blocked by missing ARM/newlib reent.h. The readback module and driver are now host-compiled with fake HAL/RTOS
(see startup diagnostics below); the complete ARM firmware build is unverified.

For the next physical run: record this snapshot, confirm left turn produces
positive Pi yaw/r, then record actual +/-90 degree turns with original TEL logs
and independent angle labels. Encoder sign needs a separate forward-wheel test.

## Startup/I2C diagnostics update

The driver now prefers the last successful address (initially HAL 0x52, 7-bit
0x29). It tries the other address only after HAL_ERROR, not HAL_BUSY/TIMEOUT.
If neither succeeds, the next retry starts with the previous preferred address.
The 700ms power-on wait runs once per Attach; repeated address probes do not
repeat that wait. Init uses a valid detection result instead of probing again.
Delays yield using osDelay when the FreeRTOS scheduler is running and retain
HAL_Delay before scheduler start. I2C HAL calls themselves remain blocking.

Add these Expressions after Clean/Build/Debug:

```
g_imu_stage
g_imu_probe_attempts
g_imu_init_attempts
g_imu_i2c_fault_count
g_imu_i2c_first_fault
g_imu_i2c_last_fault
g_imu_task_status
g_imu_config_latest
```

Stage: 0 idle, 1 power-on wait, 2 detection, 3 configuration, 4 initialized,
5 streaming read, 6 recovery check/retry wait. Stage alone is NOT an initialized/healthy flag.

Each fault snapshot contains:

- valid=1 only after publication; ignore zero-initialized status when valid=0.
- operation: 1 probe, 2 register read, 3 write, 4 unexpected CHIP_ID/mode.
- reg: register number, or 255 for address probe.
- address_7bit: 41 decimal means 0x29, 40 decimal means 0x28.
- hal_status: 0 OK, 1 ERROR, 2 BUSY, 3 TIMEOUT.
- i2c_state/lock/error_code and SR2 busy_flag.
- scl_level/sda_level: GPIO PB8/PB9 level (0/1); 255 if not I2C1.
- timestamp_ms and fault_number.

`first_fault` persists across later successes/retries until task Attach/MCU
reset. To capture a new first fault in the same run, clear only its valid field
in debugger before reproducing; last_fault keeps the latest failure. Both
snapshots have one writer (the IMU task), and valid is published last. Watch
while stopped for a consistent view. Errors at an absent fallback address may
be normal; distinguish by operation/address and subsequent initialized state.

These are snapshots after HAL returns. HAL may already clear AF or release its
lock before returning, so they are not full on-wire traces or proof of a NACK.
Bounded I2C1 bus recovery is now attempted after failures as described below.
A persistent bus fault still requires diagnosis; host tests do not establish
that the intermittent hardware failure is fixed.

Additional host regression (actual imu.c + imu_config.c against fake HAL/RTOS):

```bash
gcc -std=c11 -Wall -Wextra -Werror -Itests/stubs -IBosch_update/Core/Inc \
  tests/test_imu_startup.c Bosch_update/Core/Src/imu.c \
  Bosch_update/Core/Src/imu_config.c -o /tmp/test_imu_startup
/tmp/test_imu_startup
```

This covers preferred/fallback addresses, caching the successful address,
BUSY/TIMEOUT avoiding fallback, one startup wait, no redundant init probe,
pre-scheduler/task delay selection, first-fault persistence after success,
probe/read/write/verification failures and sampled bus levels. HAL and RTOS
are mocked, and the infinite task loop is not run. A full ARM build and board
power-cycle/debug-reset testing in CubeIDE remain required.


## Recovery for BUSY with SDA held low

Hardware fault evidence supplied on the board: operation=1 (address probe),
address=0x29, HAL_BUSY, state=0x20 READY, Lock=0 unlocked, ErrorCode=0x20 TIMEOUT,
SR2=2 (BUSY), SCL=1, SDA=0. This locates a held bus rather than a software HAL
lock. It can occur after a debug reset interrupts I2C, but does not prove that
the BNO is the only possible source holding SDA low (a short can do so too).

After failed detection/configuration or 10 consecutive streaming failures, the
IMU task now checks recovery eligibility. Only I2C1/PB8/PB9 with HAL READY and
unlocked is eligible. No action is taken after success or idle high lines with
BUSY clear. An active/locked HAL transfer is not interrupted. Only the IMU task
may use this bus; this is not a multi-master recovery implementation.

Recovery: HAL DeInit disables I2C and releases pins; pins become open-drain
outputs with no internal pullups. SET releases a pin, it does not drive high.
Wait up to about 5ms for SCL high, pulse SCL at most 9 times until SDA releases,
then generate a STOP. Restore released GPIO and reset/reinitialize I2C1 (MSP
restores AF_OD). Even after a timeout the peripheral restore is attempted.
A STOP setup can add one SCL rising edge beyond the counted recovery pulses.
No unbounded wait: typical GPIO clear tens of ms, bounded to under ~100ms at
1ms tick in the tested scenarios, subject to scheduler delays. Retry still has
backoff. No sensor power switching is attempted, and fixed wiring shorts or a
permanently held SCL/SDA must not be expected to recover.

New source `Core/Src/imu_bus_recovery.c` must appear in CubeIDE's Clean/Build
log, together with imu.c and imu_config.c. Do not flash stale Debug binaries.

Add `g_imu_bus_recovery` in Expressions:
- attempts/successes/failures/skipped: recovery counters, not sensor read counts.
- pulses: 0..9 for the most recent attempt, excluding STOP setup.
- scl_before/sda_before and scl_after/sda_after: sampled line levels.
- trigger_status, last_status, timestamp_ms; in_progress=0 after completion.

HAL_OK recovery only means bus released/reinitialization succeeded (or no
recovery was needed). Sensor health still requires initialized=1 and growing
successful_reads. Fault_count can be nonzero after a successful recovery;
first_fault intentionally remains recorded. Watch for counts stabilizing,
successful_reads increasing, and recovery successes versus failures.

Host recovery regression:

```bash
gcc -std=c11 -Wall -Wextra -Werror -Itests/stubs -IBosch_update/Core/Inc \
  tests/test_imu_bus_recovery.c Bosch_update/Core/Src/imu_bus_recovery.c \
  -o /tmp/test_imu_bus_recovery
/tmp/test_imu_bus_recovery
```

Tests use the actual recovery algorithm with a GPIO/HAL/RTOS model: release
after three clocks, permanently low SDA (max9), permanently low SCL, internal
BUSY with high lines, idle no-op, active/locked transfer gating, DeInit/Init
failures. Startup tests mock the recovery entry point and do not execute the
infinite IMU task. None of this is a physical waveform or ARM firmware build.
The next board test must repeat debug-reset with unchanged IMU power and retain
first_fault plus g_imu_bus_recovery if it fails.

## Runtime clock snapshot

For the repeated ~2% source/host clock-rate mismatch, the same branch now
captures `g_clock_config_latest` once after peripheral setup. It is read-only;
source clocks/timers and existing I2C recovery are unchanged. See
[CLOCK_DIAGNOSTICS.md](CLOCK_DIAGNOSTICS.md) for expected fields and limits.
