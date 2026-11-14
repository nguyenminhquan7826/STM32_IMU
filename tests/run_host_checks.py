"""Compile actual firmware modules against fake HAL/RTOS; no board required.
Optional --receiver/--audit cross-checks actual Communication_Task frames on Pi.
This is not an ARM firmware build or an electrical/physical sensor test.
"""
import argparse,json,subprocess,tempfile
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
SRC='Bosch_update/Core/Src/'
CASES={
 'clock':['tests/test_clock_diag.c',SRC+'clock_diag.c'],
 'imu_startup':['tests/test_imu_startup.c',SRC+'imu.c',SRC+'imu_config.c'],
 'i2c_recovery':['tests/test_imu_bus_recovery.c',SRC+'imu_bus_recovery.c'],
 'imu_config':['tests/test_imu_config.c',SRC+'imu_config.c'],
 'encoder_counter':['tests/test_encoder_counter.c',SRC+'encoder_counter.c'],
 'encoder_task':['tests/test_encoder_task.c',SRC+'encoder.c',SRC+'encoder_counter.c'],
 'telemetry_format':['tests/test_telemetry_format.c',SRC+'telemetry_format.c'],
 'communication_v2':['tests/test_communication_task.c',SRC+'communication.c',SRC+'telemetry_format.c'],
 'communication_v1':['tests/test_communication_task.c',SRC+'communication.c',SRC+'telemetry_format.c']}
def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--receiver');p.add_argument('--audit');p.add_argument('--sanitize',action='store_true');a=p.parse_args()
 if bool(a.receiver)!=bool(a.audit):p.error('--receiver and --audit must be supplied together')
 with tempfile.TemporaryDirectory(prefix='stm32-host-') as tmp:
  tmp=Path(tmp)
  for name,files in CASES.items():
   cmd=['gcc','-std=c11','-Wall','-Wextra','-Werror','-Itests/stubs','-IBosch_update/Core/Inc']
   if a.sanitize:cmd+=['-fsanitize=address,undefined']
   if name=='communication_v1':cmd+=['-DTELEMETRY_PROTOCOL_VERSION=1']
   subprocess.run(cmd+files+['-o',str(tmp/name)],cwd=ROOT,check=True)
   r=subprocess.run([str(tmp/name)],capture_output=True,text=True,check=True)
   print(name+': PASS',flush=True)
   if name.startswith('communication_') and a.receiver:
    # Preserve exactly one generated frame, without print-added blank lines.
    log=tmp/(name+'.log');log.write_bytes(r.stdout.replace('\r\n','\n').replace('\n','\r\n').encode())
    csv=tmp/(name+'.csv');report=tmp/(name+'.json')
    x=subprocess.run([str(Path(a.receiver).resolve()),'--input',str(log),'--gyro-sign','1','--diagnostics'],capture_output=True,text=True,check=True)
    csv.write_text(x.stdout)
    subprocess.run(['python3',str(Path(a.audit).resolve()),str(log),str(csv),'--gyro-sign','1','--output',str(report)],capture_output=True,text=True,check=True)
    assert json.loads(report.read_text())['passed']
    print(name+' C task -> C++ -> independent audit: PASS',flush=True)
if __name__=='__main__':main()
