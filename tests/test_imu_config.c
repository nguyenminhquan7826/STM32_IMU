#include "imu_config.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static uint8_t registers[256];
static int calls, fail_at;
uint32_t HAL_GetTick(void) { return 1234; }
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *i, uint16_t addr,
                                   uint16_t reg, uint16_t size, uint8_t *out,
                                   uint16_t len, uint32_t timeout) {
  (void)i;
  assert(addr == 0x52);
  assert(size == 1);
  assert(timeout == 25);
  calls++;
  if (calls == fail_at)
    return HAL_BUSY;
  memcpy(out, registers + reg, len);
  return HAL_OK;
}
int main(void) {
  I2C_HandleTypeDef bus;
  BNO055_HandleTypeDef d = {0};
  BNO055_ConfigSnapshot_t s;
  d.i2c = &bus;
  d.address = 0x52;
  d.initialized = 1;
  registers[0] = 0xA0;
  registers[7] = 0;
  registers[0x3B] = 0;
  registers[0x3D] = 12;
  registers[0x3E] = 0;
  registers[0x41] = 0x24;
  registers[0x42] = 0;
  assert(BNO055_ReadConfig(&d, &s) == HAL_OK);
  assert(s.valid && s.expected_units_and_ndof);
  assert(s.chip_id == 0xA0 && s.axis_map_config == 0x24 &&
         s.axis_map_sign == 0 && s.i2c_address_7bit == 0x29 &&
         s.timestamp_ms == 1234);
  registers[0x3B] = 0x80;
  assert(BNO055_ReadConfig(&d, &s) == HAL_OK);
  assert(s.valid && !s.expected_units_and_ndof);
  for (int n = 1; n <= 3; n++) {
    calls = 0;
    fail_at = n;
    assert(BNO055_ReadConfig(&d, &s) == HAL_BUSY);
    assert(!s.valid && s.last_status == HAL_BUSY);
  }
  calls = 0;
  fail_at = 0;
  registers[7] = 1;
  assert(BNO055_ReadConfig(&d, &s) == HAL_ERROR);
  assert(!s.valid && calls == 1);
  calls = 0;
  assert(BNO055_ReadConfig(NULL, &s) == HAL_ERROR);
  assert(!s.valid && calls == 0);
  assert(BNO055_ReadConfig(&d, NULL) == HAL_ERROR);
  puts("Read-only BNO config decoding, bad mode, page and transport failures "
       "PASS");
}
