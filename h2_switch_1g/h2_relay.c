#include "h2_relay.h"
#include "uartdrv.h"
#include "sl_uartdrv_instances.h"
#include "em_gpio.h"
#include "pin_config.h"

#ifndef H2_SWITCH_GANG_COUNT
#define H2_SWITCH_GANG_COUNT 1
#endif

#define RELAY_CMD_SET  0x02

static h2_relay_state_t relay_state[H2_SWITCH_GANG_COUNT];
static uint8_t seq;

static uint16_t crc16_modbus(const uint8_t *data, uint8_t len)
{
  uint16_t crc = 0xFFFF;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 1) {
        crc = (crc >> 1) ^ 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

static void relay_send_set(uint8_t gang, h2_relay_state_t state)
{
  uint8_t buf[14];
  uint8_t payload_len = 4;

  // Header
  buf[0] = 0x55;
  buf[1] = 0x80;
  buf[2] = 0x02;
  buf[3] = seq++;
  buf[4] = RELAY_CMD_SET;
  buf[5] = 0x00;
  buf[6] = payload_len;

  // Header checksum: makes sum of bytes 0-7 equal 0 mod 256
  uint8_t hdr_sum = 0;
  for (uint8_t i = 0; i < 7; i++) {
    hdr_sum += buf[i];
  }
  buf[7] = (uint8_t)(0x100 - hdr_sum);

  // Payload
  buf[8]  = gang + 1;  // 1-indexed gang number
  buf[9]  = 0x00;
  buf[10] = 0x55;
  buf[11] = (state == H2_RELAY_ON) ? 0x01 : 0x00;

  // CRC-16/MODBUS over bytes 0..11, stored little-endian
  uint16_t crc = crc16_modbus(buf, 12);
  buf[12] = (uint8_t)(crc & 0xFF);
  buf[13] = (uint8_t)(crc >> 8);

  UARTDRV_TransmitB(sl_uartdrv_eusart_wall_uart_handle, buf, sizeof(buf));
}

void h2_relay_init(void)
{
  for (uint8_t i = 0; i < H2_SWITCH_GANG_COUNT; i++) {
    relay_state[i] = H2_RELAY_OFF;
  }
  GPIO_PinModeSet(WALL7_PORT, WALL7_PIN, gpioModePushPull, 1);
}

void h2_relay_set(uint8_t gang, h2_relay_state_t state)
{
  if (gang >= H2_SWITCH_GANG_COUNT) {
    return;
  }
  relay_state[gang] = state;
  relay_send_set(gang, state);
}

h2_relay_state_t h2_relay_get(uint8_t gang)
{
  if (gang >= H2_SWITCH_GANG_COUNT) {
    return H2_RELAY_OFF;
  }
  return relay_state[gang];
}

h2_relay_state_t h2_relay_toggle(uint8_t gang)
{
  if (gang >= H2_SWITCH_GANG_COUNT) {
    return H2_RELAY_OFF;
  }
  relay_state[gang] = (relay_state[gang] == H2_RELAY_ON) ? H2_RELAY_OFF : H2_RELAY_ON;
  relay_send_set(gang, relay_state[gang]);
  return relay_state[gang];
}