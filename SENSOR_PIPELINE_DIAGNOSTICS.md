# Sensor pipeline diagnostics without changing hardware clocks

Update Pi `nmpc_rl/experimental-deployment` and build its receiver FIRST, then
update/flash STM32 `sensor-config-diagnostics`. No HSI/HSE/PLL/TIM4/UART baud,
mechanical calibration, gyro gain or Kalman parameters are changed here.

## Encoder corrections

- TIM3 remains TI12 encoder mode. Remove its two PWM_Start calls from main;
  Encoder_Task exclusively starts it and checks HAL status.
- If start fails, g_encoder_task_status.initialized=0, start_status records the
  error, encoder valid=0 and motorSpeed=0. Telemetry/task scheduling remain alive.
- Wait before the first sample, then compute velocity and PID dt from successive
  HAL timestamps. g_encoder_latest.interval_ms exposes the interval. A zero dt
  does not advance the counter baseline; no PID update is performed and that
  feedback is invalid. No arbitrary measured velocity smoothing is added.
- Cumulative count uses defined unsigned wrap, then a signed representation;
  counter delta assumes fewer than32768 ticks between samples. This does not
  detect an electrically disconnected encoder or calibrate actual wheel travel.
- Source milliseconds are still the original STM32 timebase. Measured dt fixes
  scheduling-window assumptions, not the observed source/host clock ratio.
- pulses_to_mps remains a legacy nominal10ms utility; Encoder_Task no longer
  uses it. Validate PID response on a bench before any powered closed-loop run.

Inspect g_encoder_task_status and g_encoder_latest while manually turning the
wheel. A valid timer start is not proof that encoder logic power/wiring is good.

## TEL version2

Current default is TEL2 at nominal25Hz (40 source ms), with TX buffer384bytes.
IMU and encoder tasks remain nominal100Hz; motor/servo task remains nominal50Hz.
The rate gives UART115200 headroom for a longer diagnostic packet. This is still
subsampled telemetry, not every IMU sample. The formatting stress fixture is259
bytes, which at25Hz is64.75kbit/s including8N1; any over-capacity frame is dropped
and counted, never partially sent. Check g_uart_tx_drop_count/truncate_count.

To use old TEL1/50Hz, build with preprocessor symbol
TELEMETRY_PROTOCOL_VERSION=1. An old Pi parser cannot read TEL2.

v1 fields through flags are retained; v2 adds9 fields BEFORE CRC:

```
gyro_x_dps,gyro_y_dps,roll_deg,pitch_deg,read_start_ms,read_end_ms,enc_interval_ms,system_status,system_error
```

Full v2 has29 comma-separated fields, including CRC. CRC-16/CCITT-FALSE covers
TEL through system_error, excluding the final comma/CRC/CRLF. Gyro is deg/s;
angles are degrees in BNO axes/convention. V2 Euler formatting has4 decimals to
preserve1/16-degree output resolution (v1 yaw retains2 decimals).

read_start/end bound the two I2C bursts (sensor block and status block).
imu_ms equals read_end, and task publication no longer re-stamps after decoding.
These are host MCU read times, NOT BNO physical sampling instants. IMU sequence,
raw gyroXYZ/roll/pitch and system status/error reach the Pi CSV for analysis.
Flags still mean fresh/software-valid; calibration or internal fusion health
is not silently inferred from flags. Raw axis names are not a vehicle-frame3D
transform. No automatic tilt compensation or sensor gain fit is applied.

USART2 receive is now armed only by Communication_Task, removing its redundant
startup arm in main. Command syntax is unchanged.

## Build and record

Refresh/Clean in CubeIDE and check all new source files are compiled:
encoder_counter.c, telemetry_format.c, plus existing imu_config.c,
imu_bus_recovery.c and clock_diag.c. Full ARM build/on-board tests are required;
committed Debug objects/binaries were not rebuilt in this environment.

Pi capture (motor power disconnected for first test):

```bash
./build/telemetry_rx --device /dev/ttyACM0 --duration 90 \
  --startup-discard-ms 1000 --gyro-sign 1 --host-timestamps --diagnostics \
  --raw-log pipeline_01.log --output pipeline_01.csv
```

Hold still10s, then slow left/right flat turns and pauses. Keep original LOG,
CSV and terminal summary. Run Pi audit_sensor_pipeline.py to compare the original
TEL against its CSV. Existing log/CSV cannot reconstruct omitted v2 fields.

## Verification scope

Host tests compile actual encoder counter and encoder task, including HAL start
failure, measured20ms PID update and zero dt; actual IMU driver register decoding
and read timing; formatter v1/v2 CRC and bounded output; actual communication
task snapshot-to-frame for both versions. The actual C task frames are decoded
by C++ and checked by an independent Python CRC/field/pose audit. Tests for
mutated fields and units detect CSV corruption. Existing readback, I2C recovery
and clock diagnostics remain tested.

HAL/RTOS/GPIO are mocked for host tests. No ARM toolchain/hardware is attached.
An end-to-end mapping pass does not certify physical measurement accuracy,
source clock rate, mounting, calibration or motor safety. Calibration/tilt and
known-angle tests remain necessary after data integrity is verified.

Run all host regression checks with:

```bash
python3 tests/run_host_checks.py --sanitize
# Optional cross-repository check with a freshly built Pi receiver:
python3 tests/run_host_checks.py --sanitize \
  --receiver /path/to/nmpc_rl/build/telemetry_rx \
  --audit /path/to/nmpc_rl/deployment_cpp14/tools/audit_sensor_pipeline.py
```

The cross-check builds the actual C Communication_Task twice (v1/v2) with fake
sensor snapshots, emits its frame, runs C++ decode and checks independent CSV
mapping/pose. It is not a claim that a physical UART/I2C measurement was made.

## Existing unrelated Python training failures

The broad Python training suite still has124 passing tests and these4 prior
failures; no training Python code was modified in this task:

- FAIL: test_user_config_d_reproduces_completed_v3_settings (test_component_learning_rates.IndependentLearningRateTests.test_user_config_d_reproduces_completed_v3_settings)
- FAIL: test_layout_has_only_two_new_output_roots_and_pinned_source (test_one_command_experiment.MinimalLayoutTests.test_layout_has_only_two_new_output_roots_and_pinned_source)
- FAIL: test_single_run_command_requires_no_paths_or_manifest (test_one_command_experiment.MinimalLayoutTests.test_single_run_command_requires_no_paths_or_manifest)
- FAIL: test_candidate_box_contains_both_in_strict_interior (test_screen_training_weights.ScreeningSpecTests.test_candidate_box_contains_both_in_strict_interior)

## BNO055 warm-restart initialization (NDOF retained)

A reset of the MCU by the debugger does not reset a separately powered BNO055.
The driver now enters CONFIG, writes SYS_TRIGGER.RST_SYS once per initialization,
waits 700 source ms, and polls CHIP_ID for at most another 500 source ms (plus
one bounded HAL transaction/delay). After CHIP_ID returns A0 it waits 50 ms,
then reapplies page, normal power, SI/degree units, temperature source and NDOF.
HAL_BUSY exits to the existing task bus-recovery path. A missing chip times out;
failed initialization retries use the existing task backoff, not a tight loop.
The normal stream never resets the sensor solely because SYS_ERR is nonzero.
No SYSCLK, timer, serial baud, mounting sign or gyro gain is changed.

Soft reset clears volatile sensor state, including calibration/offset state.
Allow calibration to settle again; no stored offsets are restored by this change.
Magnetometer calibration is not required for packet acceptance. NDOF still uses
its magnetometer internally, even when the Pi does not consume raw mag values.

Debugger fields in `g_imu_latest`:

- `valid`: the last sample was read successfully over I2C. Use timestamp freshness
  as well; this field alone does not guarantee a current or accurate sample.
- `system_status`, `system_error`: original BNO055 values, also retained in TEL2.
- `fusion_healthy`: status is 5 AND error is 0. This is only a reported-health
  indicator, not proof of physical accuracy, full calibration or yaw stability.

`g_imu_task_status.initialized` means configuration succeeded, not that fusion
has already settled. Fusion health is evaluated on every successful sample.
An error-9 sample remains available for diagnosis with `valid=1` and
`fusion_healthy=0`. TEL2 layout and its freshness flags are unchanged; on Pi,
check `imu_system_status==5` and `imu_system_error==0` explicitly before accepting
fusion for a control experiment. This does not introduce a controller interlock.

### Board validation

1. In the firmware clone, `git switch sensor-config-diagnostics` and
   `git pull --ff-only`. Verify CubeIDE project Location points to that clone;
   Clean Project, Build, and flash the new ELF.
2. With motor power disabled, do one full power-on test. Wait a few seconds,
   observe increasing `successful_reads`, and record SYS_STATUS/SYS_ERR.
3. Keep power connected and stop/start debugging five times, resuming execution
   each time. Initialization may take about two seconds. Record whether reads
   resume, health becomes 5/0, and whether I2C faults/recovery recur.
4. Capture each case with a fresh filename, e.g.:

```sh
./build/telemetry_rx --device /dev/ttyACM0 --duration 30 \
  --startup-discard-ms 1000 --gyro-sign 1 --host-timestamps --diagnostics \
  --raw-log imu_warm_01.log --output imu_warm_01.csv
```

Host verification: all nine firmware fake-HAL test groups pass with ASan/UBSan,
including delayed reset completion, never-ready timeout, reset-write failure,
retained fusion-error reset modeling, and separate transport/fusion health.
Actual C task TEL1/TEL2 output also passes the C++ receiver/Python mapping audit.
These tests do not prove that real SYS_ERR=9 is cleared by soft reset. A CubeIDE
ARM build and the repeated board tests above remain necessary.
