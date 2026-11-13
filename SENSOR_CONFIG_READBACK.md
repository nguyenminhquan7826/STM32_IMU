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
headers was blocked by missing ARM/newlib reent.h. Only the isolated readback
module is host-compiled and tested; the complete firmware build is unverified.

For the next physical run: record this snapshot, confirm left turn produces
positive Pi yaw/r, then record actual +/-90 degree turns with original TEL logs
and independent angle labels. Encoder sign needs a separate forward-wheel test.
