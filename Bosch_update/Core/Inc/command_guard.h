#ifndef COMMAND_GUARD_H
#define COMMAND_GUARD_H
#include <stddef.h>
#include <stdint.h>
#define COMMAND_TIMEOUT_MS 300U
#define COMMAND_MAX_SPEED_MPS 0.5f
#define COMMAND_SERVO_CENTER 75
#define COMMAND_SERVO_MIN 45
#define COMMAND_SERVO_MAX 105
/* Zero is accepted; other codes are explicit rejection/stop reasons. */
enum {
  COMMAND_OK,
  COMMAND_PARSE,
  COMMAND_RANGE,
  COMMAND_DISARMED,
  COMMAND_SEQUENCE,
  COMMAND_TIMEOUT,
  COMMAND_SENSOR
};
typedef struct {
  uint32_t sequence, last_ms, accepted, rejected, timeouts, stop_generation;
  float speed;
  int servo;
  uint8_t armed, have_sequence, reason;
} CommandGuard;
uint16_t Command_CRC(const char *text, size_t len);
void CommandGuard_Init(CommandGuard *s);
void CommandGuard_Stop(CommandGuard *s, uint8_t reason);
int CommandGuard_Tick(CommandGuard *s, uint32_t now);
int CommandGuard_Process(CommandGuard *s, const char *line, uint32_t now,
                         uint32_t *seq);
#endif
