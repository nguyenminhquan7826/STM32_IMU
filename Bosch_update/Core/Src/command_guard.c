#include "command_guard.h"
#include <errno.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
uint16_t Command_CRC(const char *text, size_t len) {
  uint16_t crc = 0xffffU;
  size_t i;
  unsigned b;
  for (i = 0; i < len; i++) {
    crc ^= (uint16_t)(unsigned char)text[i] << 8;
    for (b = 0; b < 8; b++)
      crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U)
                            : (uint16_t)(crc << 1);
  }
  return crc;
}
void CommandGuard_Init(CommandGuard *s) {
  memset(s, 0, sizeof(*s));
  s->servo = COMMAND_SERVO_CENTER;
}
void CommandGuard_Stop(CommandGuard *s, uint8_t reason) {
  s->stop_generation++;
  s->armed = 0;
  s->speed = 0;
  s->servo = COMMAND_SERVO_CENTER;
  s->reason = reason;
}
int CommandGuard_Tick(CommandGuard *s, uint32_t now) {
  if (s->armed && (uint32_t)(now - s->last_ms) >= COMMAND_TIMEOUT_MS) {
    s->timeouts++;
    CommandGuard_Stop(s, COMMAND_TIMEOUT);
    return 1;
  }
  return 0;
}
static int reject(CommandGuard *s, int reason) {
  s->rejected++;
  return reason;
}
static int decimal(const char *p, unsigned long *v) {
  char *end;
  const char *q = p;
  if (!*q)
    return 0;
  for (; *q; q++)
    if (*q < '0' || *q > '9')
      return 0;
  errno = 0;
  *v = strtoul(p, &end, 10);
  return !errno && !*end;
}
int CommandGuard_Process(CommandGuard *s, const char *line, uint32_t now,
                         uint32_t *seq) {
  char buf[128], *fields[7], *p, *comma, *end;
  size_t len;
  unsigned i;
  unsigned long n, crc, angle;
  float speed;
  *seq = 0;
  CommandGuard_Tick(s, now);
  if (!line || (len = strlen(line)) >= sizeof(buf))
    return reject(s, COMMAND_PARSE);
  memcpy(buf, line, len + 1);
  p = buf;
  for (i = 0; i < 7; i++) {
    fields[i] = p;
    comma = strchr(p, ',');
    if (i < 6) {
      if (!comma)
        return reject(s, COMMAND_PARSE);
      *comma = 0;
      p = comma + 1;
    } else if (comma)
      return reject(s, COMMAND_PARSE);
  }
  if (strcmp(fields[0], "CTL") || strcmp(fields[1], "1") ||
      strlen(fields[6]) != 4)
    return reject(s, COMMAND_PARSE);
  for (i = 0; i < 4; i++)
    if (!((fields[6][i] >= '0' && fields[6][i] <= '9') ||
          (fields[6][i] >= 'A' && fields[6][i] <= 'F') ||
          (fields[6][i] >= 'a' && fields[6][i] <= 'f')))
      return reject(s, COMMAND_PARSE);
  crc = strtoul(fields[6], &end, 16);
  if (*end || crc != Command_CRC(line, (size_t)(strrchr(line, ',') - line)) ||
      !decimal(fields[2], &n) || n > UINT32_MAX)
    return reject(s, COMMAND_PARSE);
  *seq = (uint32_t)n;
  errno = 0;
  speed = strtof(fields[4], &end);
  if (errno || end == fields[4] || *end || !isfinite(speed) || speed < 0 ||
      speed > COMMAND_MAX_SPEED_MPS || !decimal(fields[5], &angle) ||
      angle < COMMAND_SERVO_MIN || angle > COMMAND_SERVO_MAX)
    return reject(s, COMMAND_RANGE);
  if (!strcmp(fields[3], "STOP")) {
    if (speed != 0 || angle != COMMAND_SERVO_CENTER)
      return reject(s, COMMAND_RANGE);
    CommandGuard_Stop(s, COMMAND_OK);
    s->accepted++;
    return COMMAND_OK;
  }
  if (strcmp(fields[3], "ARM") && strcmp(fields[3], "RUN"))
    return reject(s, COMMAND_PARSE);
  if (s->have_sequence && ((uint32_t)(*seq - s->sequence) == 0 ||
                           (uint32_t)(*seq - s->sequence) >= 0x80000000U))
    return reject(s, COMMAND_SEQUENCE);
  if (!strcmp(fields[3], "ARM")) {
    if (s->armed || speed != 0 || angle != COMMAND_SERVO_CENTER)
      return reject(s, COMMAND_RANGE);
    s->armed = 1;
  } else if (!s->armed)
    return reject(s, COMMAND_DISARMED);
  s->speed = speed;
  s->servo = (int)angle;
  s->sequence = *seq;
  s->have_sequence = 1;
  s->last_ms = now;
  s->accepted++;
  s->reason = COMMAND_OK;
  return COMMAND_OK;
}
