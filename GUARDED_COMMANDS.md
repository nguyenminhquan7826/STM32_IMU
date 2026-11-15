# Guarded commands for the Pi NMPC experiment

Branch sensor-config-diagnostics. Clock/NDOF/telemetry fields unchanged.
Legacy CMD,speed,servo is intentionally no longer an actuator command. Use the
paired Pi hardware_runner on nmpc_rl/experimental-deployment. telemetry_rx stays
RX-only. Full build/run/bench workflow:
https://github.com/nguyenminhquan7826/nmpc_rl/blob/experimental-deployment/deployment_cpp14/HARDWARE_EXPERIMENT.md

CTL,1,seq,ARM|RUN|STOP,speed_mps,servo_deg,CRC16/CCITT-FALSE CRLF.
CRC covers everything before final comma. ACK,1,echo_seq,armed,reason,stm32_ms,
last_accepted_seq,CRC CRLF. ACK confirms accepted request, not physical motion.
Default disarmed. ARM/STOP neutral0m/s,75deg. RUN forward0..0.5m/s,45..105deg,
requires newer uint32 sequence and armed. Watchdog300 source ms. Rejected packets
never refresh it. STOP accepts any sequence; next ARM/RUN must remain newer than
last accepted ARM/RUN. Pi obtains that sequence from the initial STOP ACK.

Motor gate independently checks timeout, fresh encoder and fresh IMU with
fusion5/0. PWM0 on stop/disarm/zero target; no reliance on PID integral decay.
Encoder task owns PID and consumes a latched stop_generation so even a complete
STOP->ARM->RUN between encoder ticks clears PID history before restart.

Inspect g_command_guard armed/speed/servo/last_ms/sequence/reason/timeouts/
stop_generation plus g_encoder_task_status, g_imu_task_status and g_imu_latest.
Reason0accepted,1parse/CRC,2bounds,3disarmed,4sequence,5timeout,6sensor.

CubeIDE: refresh imported project, Clean/Build; check command_guard.c and
actuator_gate.c compile. Old committed generated Debug files are not the new
firmware; regenerate rather than manually editing generated makefiles. No .ioc
or pin changes. Motor task calls Control_ActuatorStep. Disable motor power while
flashing/debugging: software watchdog cannot act when the MCU is halted. PWM0
is not proof of physical braking, nor protection against a hung MCU/driver.

Host checks (not ARM build):
python3 tests/run_host_checks.py --sanitize
Optional paired checks:
python3 tests/run_host_checks.py --sanitize --receiver /path/to/telemetry_rx \
 --audit /path/to/audit_sensor_pipeline.py --command-probe /path/to/test_hardware_control

Before floor runs, bench-test actual motor stop after stopping Pi commands,
health failure and source reset, then calibrated servo sign and low-speed PID.
Hardware/AprilTag validation remains pending.
