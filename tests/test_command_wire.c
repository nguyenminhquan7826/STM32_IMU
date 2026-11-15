#include "command_guard.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
int main(void) {
  CommandGuard s;
  char line[128], payload[128];
  unsigned count = 0;
  uint32_t seq;
  CommandGuard_Init(&s);
  while (fgets(line, sizeof(line), stdin)) {
    line[strcspn(line, "\r\n")] = 0;
    assert(CommandGuard_Process(&s, line, 10U + count * 20U, &seq) ==
           COMMAND_OK);
    assert(seq == count + 1 && s.armed == (count < 2));
    snprintf(payload, sizeof(payload), "ACK,1,%lu,%u,0,%u,%lu",
             (unsigned long)seq, s.armed, 10U + count * 20U,
             (unsigned long)s.sequence);
    printf("%s,%04X\r\n", payload, Command_CRC(payload, strlen(payload)));
    count++;
  }
  assert(count == 3);
  return 0;
}
