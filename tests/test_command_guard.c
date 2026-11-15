#include "command_guard.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static char frame[128];
static const char *make(unsigned seq, const char *action, const char *speed,
                        int servo) {
  char payload[96];
  snprintf(payload, sizeof(payload), "CTL,1,%u,%s,%s,%d", seq, action, speed,
           servo);
  snprintf(frame, sizeof(frame), "%s,%04X", payload,
           Command_CRC(payload, strlen(payload)));
  return frame;
}
int main(void) {
  CommandGuard s;
  uint32_t seq;
  CommandGuard_Init(&s);
  assert(!s.armed && s.speed == 0 && s.servo == 75);
  assert(CommandGuard_Process(&s, make(1, "RUN", "0.1", 75), 10, &seq) ==
         COMMAND_DISARMED);
  assert(CommandGuard_Process(&s, make(2, "ARM", "0", 75), 10, &seq) ==
             COMMAND_OK &&
         s.armed);
  assert(CommandGuard_Process(&s, make(3, "RUN", "0.1", 90), 20, &seq) ==
             COMMAND_OK &&
         s.speed > .09f);
  assert(CommandGuard_Process(&s, make(3, "RUN", "0.2", 90), 21, &seq) ==
         COMMAND_SEQUENCE);
  assert(CommandGuard_Process(&s, make(4, "RUN", "nan", 90), 22, &seq) ==
         COMMAND_RANGE);
  assert(CommandGuard_Process(&s, make(4, "RUN", "0.6", 90), 22, &seq) ==
         COMMAND_RANGE);
  assert(CommandGuard_Process(&s, make(4, "RUN", "-0.1", 90), 22, &seq) ==
         COMMAND_RANGE);
  assert(CommandGuard_Process(&s, make(4, "RUN", "0.2", 106), 22, &seq) ==
         COMMAND_RANGE);
  assert(CommandGuard_Process(&s, "CMD,0.1,75", 22, &seq) == COMMAND_PARSE);
  make(4, "RUN", "0.2", 90);
  frame[5] = '9';
  assert(CommandGuard_Process(&s, frame, 22, &seq) == COMMAND_PARSE);
  assert(!CommandGuard_Tick(&s, 319));
  assert(CommandGuard_Tick(&s, 320));
  assert(!s.armed && s.speed == 0);
  assert(CommandGuard_Process(&s, make(5, "RUN", "0.2", 90), 321, &seq) ==
         COMMAND_DISARMED);
  assert(CommandGuard_Process(&s, make(6, "ARM", "0", 75), 0xffffff80U, &seq) ==
         COMMAND_OK);
  assert(CommandGuard_Process(&s, make(7, "RUN", "0.2", 90), 0xffffff90U,
                              &seq) == COMMAND_OK);
  assert(!CommandGuard_Tick(&s, 100));
  assert(CommandGuard_Tick(&s, 200));
  assert(CommandGuard_Process(&s, make(8, "ARM", "0", 75), 250, &seq) ==
         COMMAND_OK);
  assert(CommandGuard_Process(&s, make(0, "STOP", "0", 75), 251, &seq) ==
             COMMAND_OK &&
         !s.armed);
  unsigned generation = s.stop_generation;
  assert(CommandGuard_Process(&s, make(9, "ARM", "0", 75), 252, &seq) ==
         COMMAND_OK);
  assert(CommandGuard_Process(&s, make(10, "RUN", "0.1", 75), 253, &seq) ==
         COMMAND_OK);
  assert(CommandGuard_Process(&s, make(0, "STOP", "0", 75), 254, &seq) ==
         COMMAND_OK);
  assert(CommandGuard_Process(&s, make(11, "ARM", "0", 75), 255, &seq) ==
         COMMAND_OK);
  assert(CommandGuard_Process(&s, make(12, "RUN", "0.1", 75), 256, &seq) ==
         COMMAND_OK);
  assert(s.stop_generation == generation + 1 && s.armed);
  assert(Command_CRC("123456789", 9) == 0x29b1);
  puts("Command CRC, arm/run/stop, bounds, sequence, watchdog and rollover "
       "PASS");
}
