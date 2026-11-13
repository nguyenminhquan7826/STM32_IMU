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
5 streaming read. Stage alone is NOT an initialized/healthy flag.

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
No GPIO clock pulses, I2C reset or automatic electrical-bus recovery is added.
A persistent bus fault still requires diagnosis; this patch does not establish
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
