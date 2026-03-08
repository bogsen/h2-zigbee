#include "app/framework/include/af.h"
#include "h2_relay.h"
#include "uartdrv.h"
#include "sl_uartdrv_instances.h"
#include "em_eusart.h"
#include "em_gpio.h"
#include "pin_config.h"
#include "sl_power_manager.h"
#include <string.h>

#include "h2_variant.h"

extern uint8_t h2_active_gang_count;

#if H2_HAS_DIMMER
#define RELAY_TYPE     0x8007
#else
#define RELAY_TYPE     0x8002
#endif
#define RELAY_CMD_SET  0x02
#define RELAY_CMD_QUERY 0x03

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

// Generic packet builder + blocking transmit
// Payload must already be written to buf starting at index 8
static void relay_send_packet(uint16_t type, uint8_t cmd,
                               const uint8_t *payload, uint8_t payload_len)
{
  uint8_t buf[72];  // 8 header + max 62 payload + 2 CRC
  uint8_t total = 8 + payload_len + 2;

  // Header (big-endian type, big-endian payload_len)
  buf[0] = 0x55;
  buf[1] = (uint8_t)(type >> 8);
  buf[2] = (uint8_t)(type & 0xFF);
  buf[3] = seq++;
  buf[4] = cmd;
  buf[5] = (uint8_t)(payload_len >> 8);
  buf[6] = (uint8_t)(payload_len & 0xFF);

  // Header checksum: sum of bytes 0-7 == 0 mod 256
  uint8_t hdr_sum = 0;
  for (uint8_t i = 0; i < 7; i++) {
    hdr_sum += buf[i];
  }
  buf[7] = (uint8_t)(0x100 - hdr_sum);

  // Payload
  if (payload_len > 0 && payload != NULL) {
    memcpy(&buf[8], payload, payload_len);
  }

  // CRC-16/MODBUS over all bytes before CRC, stored little-endian
  uint16_t crc = crc16_modbus(buf, 8 + payload_len);
  buf[8 + payload_len]     = (uint8_t)(crc & 0xFF);
  buf[8 + payload_len + 1] = (uint8_t)(crc >> 8);

  // Log TX packet
  sl_zigbee_app_debug_print("TX PKT type=%02X%02X cmd=%02X plen=%d [",
                             buf[1], buf[2], buf[4], payload_len);
  uint8_t dump = (payload_len < 24) ? payload_len : 24;
  for (uint8_t i = 0; i < dump; i++) {
    sl_zigbee_app_debug_print("%02X", buf[8 + i]);
  }
  sl_zigbee_app_debug_println("]");

  // Hold EM1 during TX — EM2 would stop the EUSART HF clock and cause
  // glitch pulses on the TX line that corrupt every packet
  sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM1);

#if H2_HAS_DIMMER
  // Dimmer stock firmware: WALL2 only goes low for SET commands,
  // held for the entire exchange. ACK/query packets don't toggle WALL2.
  if (cmd == RELAY_CMD_SET) {
    GPIO_PinOutClear(WALL2_PORT, WALL2_PIN);
  }
#else
  GPIO_PinOutClear(WALL2_PORT, WALL2_PIN);
#endif

  UARTDRV_TransmitB(sl_uartdrv_eusart_wall_uart_handle, buf, total);

#if H2_HAS_DIMMER
  if (cmd == RELAY_CMD_SET) {
    GPIO_PinOutSet(WALL2_PORT, WALL2_PIN);
  }
#else
  GPIO_PinOutSet(WALL2_PORT, WALL2_PIN);
#endif

  sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM1);
}

static void relay_send_set(uint8_t gang, h2_relay_state_t state)
{
  // Payload format: [cmd_hi=0x01, cmd_lo=gang, marker=0x55, value]
  // cmd 0x0100 = gang 0 relay, cmd 0x0101 = gang 1 relay
  uint8_t payload[4];
  payload[0] = 0x01;      // cmd_hi: relay control
  payload[1] = gang;      // cmd_lo: gang index (0-based)
  payload[2] = 0x55;      // data_type marker
  payload[3] = (state == H2_RELAY_ON) ? 0x01 : 0x00;
  relay_send_packet(RELAY_TYPE, RELAY_CMD_SET, payload, sizeof(payload));
}

// ---------------------------------------------------------------------------
// UART RX state machine (byte-at-a-time, ISR-driven)
// ---------------------------------------------------------------------------

#define RX_BUF_SIZE 72  // 8 header + max payload + 2 CRC
#define RX_MAX_PAYLOAD 56

typedef enum {
  RX_WAIT_SYNC,
  RX_READ_HEADER,
  RX_READ_PAYLOAD,
  RX_READ_CRC,
} rx_state_t;

static uint8_t rx_byte;                    // 1-byte DMA target
static uint8_t rx_buf[RX_BUF_SIZE];        // accumulation buffer
static uint8_t rx_pos;
static rx_state_t rx_state;
static uint16_t rx_payload_len;

static uint8_t rx_dispatch_buf[RX_BUF_SIZE]; // double-buffer for main loop
static uint16_t rx_dispatch_len;             // total packet length
static volatile bool rx_packet_ready;

static h2_relay_rx_callback_t rx_callback;

static void uart_rx_start(void);

static void uart_rx_cb(UARTDRV_Handle_t handle,
                        Ecode_t transferStatus,
                        uint8_t *data,
                        UARTDRV_Count_t transferCount)
{
  (void)handle;
  (void)transferCount;

  if (transferStatus != ECODE_EMDRV_UARTDRV_OK) {
    rx_state = RX_WAIT_SYNC;
    uart_rx_start();
    return;
  }

  uint8_t b = rx_byte;

  switch (rx_state) {
    case RX_WAIT_SYNC:
      if (b == 0x55) {
        rx_buf[0] = b;
        rx_pos = 1;
        rx_state = RX_READ_HEADER;
      }
      break;

    case RX_READ_HEADER:
      rx_buf[rx_pos++] = b;
      if (rx_pos >= 8) {
        // Validate header checksum: sum of bytes 0-7 == 0 mod 256
        uint8_t sum = 0;
        for (uint8_t i = 0; i < 8; i++) {
          sum += rx_buf[i];
        }
        if (sum != 0) {
          rx_state = RX_WAIT_SYNC;
          break;
        }
        rx_payload_len = ((uint16_t)rx_buf[5] << 8) | rx_buf[6];
        if (rx_payload_len > RX_MAX_PAYLOAD) {
          rx_state = RX_WAIT_SYNC;
          break;
        }
        if (rx_payload_len == 0) {
          rx_state = RX_READ_CRC;
        } else {
          rx_state = RX_READ_PAYLOAD;
        }
      }
      break;

    case RX_READ_PAYLOAD:
      rx_buf[rx_pos++] = b;
      if (rx_pos >= 8 + rx_payload_len) {
        rx_state = RX_READ_CRC;
      }
      break;

    case RX_READ_CRC:
      rx_buf[rx_pos++] = b;
      if (rx_pos >= 8 + rx_payload_len + 2) {
        // Validate CRC-16/MODBUS
        uint16_t pkt_len = 8 + rx_payload_len;
        uint16_t expected_crc = (uint16_t)rx_buf[pkt_len]
                              | ((uint16_t)rx_buf[pkt_len + 1] << 8);
        uint16_t calc_crc = crc16_modbus(rx_buf, pkt_len);
        if (calc_crc == expected_crc && !rx_packet_ready) {
          memcpy(rx_dispatch_buf, rx_buf, rx_pos);
          rx_dispatch_len = rx_pos;
          rx_packet_ready = true;
        }
        rx_state = RX_WAIT_SYNC;
      }
      break;
  }

  uart_rx_start();
}

static void uart_rx_start(void)
{
  UARTDRV_Receive(sl_uartdrv_eusart_wall_uart_handle, &rx_byte, 1, uart_rx_cb);
}

void h2_relay_set_rx_callback(h2_relay_rx_callback_t cb)
{
  rx_callback = cb;
}

void h2_relay_process_rx(void)
{
  if (!rx_packet_ready) {
    return;
  }

  // Log raw packet header + first bytes for debugging
  {
    uint16_t plen = ((uint16_t)rx_dispatch_buf[5] << 8) | rx_dispatch_buf[6];
    sl_zigbee_app_debug_print("RX PKT type=%02X%02X cmd=%02X plen=%d [",
                               rx_dispatch_buf[1], rx_dispatch_buf[2],
                               rx_dispatch_buf[4], plen);
    uint16_t dump = (plen < 24) ? plen : 24;
    for (uint16_t i = 0; i < dump; i++) {
      sl_zigbee_app_debug_print("%02X", rx_dispatch_buf[8 + i]);
    }
    sl_zigbee_app_debug_println("]");
  }

  // Send ACK echoing the wall module's sequence number,
  // but only for measurement reports (cmd=0x05), not for wall ACKs (cmd=0x03)
  if (rx_dispatch_buf[4] == 0x05) {
    uint8_t rx_seq = rx_dispatch_buf[3];
    uint8_t saved_seq = seq;
    seq = rx_seq;
    uint8_t ack_payload[3] = {0, 0, 0};
    relay_send_packet(RELAY_TYPE, RELAY_CMD_QUERY, ack_payload, 3);
    seq = saved_seq;
  }

  if (rx_callback != NULL) {
    uint16_t payload_len = ((uint16_t)rx_dispatch_buf[5] << 8) | rx_dispatch_buf[6];
    const uint8_t *payload = &rx_dispatch_buf[8];

    // Parse variable-length command entries from payload
    // Wire format: [cmd_hi(1), cmd_lo(1), marker(1), data(N)]
    // N determined by cmd_hi via DATA_TYPE_SIZE_LOOKUP
    uint16_t off = 0;
    while (off + 3 <= payload_len) {
      uint8_t cmd_hi = payload[off];
      uint8_t cmd_lo = payload[off + 1];
      uint8_t marker = payload[off + 2];

      // DATA_TYPE_SIZE_LOOKUP: cmd_hi → data size
      uint8_t data_size;
      switch (cmd_hi) {
        case 0x01: case 0x04: data_size = 1; break;
        case 0x00: case 0x02: case 0x03:
        case 0x05: case 0x06: data_size = 4; break;
        default:
          // Variable length: size in payload byte[3]
          if (off + 3 < payload_len) {
            data_size = payload[off + 3];
          } else {
            data_size = 0;  // can't determine size, will break below
          }
          break;
      }

      if (off + 3 + data_size > payload_len) break;

      // Data is big-endian on wire — byte-reverse for ARM little-endian
      uint8_t data_le[8];
      const uint8_t *src = &payload[off + 3];
      if (data_size == 4) {
        data_le[0] = src[3];
        data_le[1] = src[2];
        data_le[2] = src[1];
        data_le[3] = src[0];
      } else if (data_size == 1) {
        data_le[0] = src[0];
      } else {
        // Generic reverse
        for (uint8_t i = 0; i < data_size && i < sizeof(data_le); i++) {
          data_le[i] = src[data_size - 1 - i];
        }
      }

      uint16_t cmd_id = ((uint16_t)cmd_hi << 8) | cmd_lo;
      rx_callback(cmd_id, marker, data_le, data_size);

      off += 3 + data_size;
    }
  }

  rx_packet_ready = false;
}

// ---------------------------------------------------------------------------
// TX commands: init handshake + measurement query
// ---------------------------------------------------------------------------

void h2_relay_send_init(void)
{
  // Query 1: general status (type=0x0080, cmd=0x03, no payload)
  relay_send_packet(0x0080, RELAY_CMD_QUERY, NULL, 0);
  // Query 2: relay state (type=0x8001, cmd=0x03, no payload)
  relay_send_packet(0x8001, RELAY_CMD_QUERY, NULL, 0);

#if H2_HAS_DIMMER
  // Dimmer-specific init handshake — matches stock firmware UART_INIT sequence:
  // 1) INIT_QUERY_3: gang query + level + transition (off, level 0)
  {
    uint8_t q3[18] = {
      0x01, 0x00, 0x55, 0x00,                          // cmd 0x0100, marker 0x55, state=0
      0x00, 0x1A, 0x80, 0x00, 0x00, 0x00, 0x00,        // cmd 0x001A, marker 0x80, level=0
      0x00, 0x17, 0x80, 0x00, 0x00, 0x00, 0x00          // cmd 0x0017, marker 0x80, transition=0
    };
    relay_send_packet(RELAY_TYPE, RELAY_CMD_SET, q3, sizeof(q3));
  }
  // 2) + 3) are sent by h2_relay_configure_dimmer() from app.c after opple init
#endif
}

void h2_relay_send_measurement_query(uint8_t gang)
{
  // Existing working format: [0x01, 0x0C, 0x55, 0x01, float(0.0)]
  uint8_t payload[8];
  payload[0] = 0x01;      // cmd_hi
  payload[1] = 0x0C;      // cmd_lo: measurement attribute
  payload[2] = 0x55;      // data_type marker
  payload[3] = 0x01;      // rw_flag = read/query
  payload[4] = 0x00;      // float(0.0) BE
  payload[5] = 0x00;
  payload[6] = 0x00;
  payload[7] = 0x00;
  relay_send_packet(RELAY_TYPE, RELAY_CMD_SET, payload, sizeof(payload));
}

// ---------------------------------------------------------------------------
// Dimmer level control
// ---------------------------------------------------------------------------

#if H2_HAS_DIMMER
static uint8_t dimmer_level[H2_SWITCH_GANG_COUNT];
static uint16_t on_off_transition_ms = 500;  // auto-set to 0 when min==max brightness

static uint8_t brightness_curve(uint8_t level)
{
  float f = (float)level / 254.0f;
  float d = f * f;  // gamma = 2.0
  int result = (int)(d * 254.0f);
  if (result < 1) result = 1;
  if (result > 254) result = 254;
  return (uint8_t)result;
}

void h2_relay_set_level(uint8_t gang, uint8_t level)
{
  if (gang >= h2_active_gang_count) return;
  if (level < 1) level = 1;
  if (level > 254) level = 254;
  if (dimmer_level[gang] == level) return;  // skip if unchanged
  dimmer_level[gang] = level;

  uint8_t phase = brightness_curve(level);

  // Level + fixed short transition for smooth stepping
  uint8_t payload[14];
  // Entry 1: level (cmd_id 0x001A, marker 0x80, 4-byte BE phase)
  payload[0]  = 0x00; payload[1]  = 0x1A; payload[2]  = 0x80;
  payload[3]  = 0x00; payload[4]  = 0x00; payload[5]  = 0x00; payload[6]  = phase;
  // Entry 2: transition (cmd_id 0x0017, marker 0x80, 100ms fixed)
  payload[7]  = 0x00; payload[8]  = 0x17; payload[9]  = 0x80;
  payload[10] = 0x00; payload[11] = 0x00; payload[12] = 0x00; payload[13] = 0x64;
  relay_send_packet(RELAY_TYPE, RELAY_CMD_SET, payload, sizeof(payload));
}

uint8_t h2_relay_get_level(uint8_t gang)
{
  if (gang >= h2_active_gang_count) return 0;
  return dimmer_level[gang];
}

void h2_relay_set_brightness_range(uint8_t min_pct, uint8_t max_pct)
{
  // Convert percentage (0-100) to phase value via brightness curve
  uint8_t min_phase = (min_pct > 0) ? brightness_curve(min_pct * 254 / 100) : 1;
  uint8_t max_phase = (max_pct < 100) ? brightness_curve(max_pct * 254 / 100) : 254;
  if (min_phase < 1) min_phase = 1;
  if (max_phase < min_phase) max_phase = min_phase;

  // When min == max brightness, on/off transition is pointless — make it instant
  on_off_transition_ms = (min_pct == max_pct) ? 0 : 500;

  uint8_t payload[14] = {
    0x00, 0x18, 0x80, 0x00, 0x00, 0x00, min_phase,
    0x00, 0x19, 0x80, 0x00, 0x00, 0x00, max_phase
  };
  relay_send_packet(RELAY_TYPE, RELAY_CMD_SET, payload, sizeof(payload));
}

void h2_relay_set_phase_mode(uint8_t mode)
{
  // Opple: 0=forward, 1=reverse → register: 0x00=leading edge, 0x01=trailing edge
  uint8_t reg_val = (mode == 1) ? 0x01 : 0x00;
  uint8_t payload[7] = {
    0x00, 0x1B, 0x80, 0x00, 0x00, 0x00, reg_val
  };
  relay_send_packet(RELAY_TYPE, RELAY_CMD_SET, payload, sizeof(payload));
}
#endif

// ---------------------------------------------------------------------------

void h2_relay_init(void)
{
  for (uint8_t i = 0; i < H2_SWITCH_GANG_COUNT; i++) {
    relay_state[i] = H2_RELAY_OFF;
#if H2_HAS_DIMMER
    dimmer_level[i] = 254;  // default to max brightness
#endif
  }
  GPIO_PinModeSet(WALL7_PORT, WALL7_PIN, gpioModePushPull, 1);
  GPIO_PinModeSet(WALL2_PORT, WALL2_PIN, gpioModePushPull, 1);
  GPIO_PinModeSet(WALL3_PORT, WALL3_PIN, gpioModeInputPull, 1);
  uart_rx_start();
}

void h2_relay_set(uint8_t gang, h2_relay_state_t state)
{
  if (gang >= h2_active_gang_count) {
    return;
  }
  relay_state[gang] = state;
#if H2_HAS_DIMMER
  // Dimmer uses INIT_QUERY_3 format: gang query + level + transition
  // The gang query entry (0x0100) is required for the wall module to
  // actually enable/disable the triac output.
  uint8_t phase = (state == H2_RELAY_ON)
    ? brightness_curve(dimmer_level[gang])
    : 0x00;  // level 0 = off
  uint8_t payload[18];
  // Entry 1: gang query (cmd_id 0x0100, marker 0x55, 1-byte state)
  payload[0]  = 0x01;
  payload[1]  = gang;
  payload[2]  = 0x55;
  payload[3]  = (state == H2_RELAY_ON) ? 0x01 : 0x00;
  // Entry 2: level (cmd_id 0x001A, marker 0x80, 4-byte BE)
  payload[4]  = 0x00; payload[5]  = 0x1A; payload[6]  = 0x80;
  payload[7]  = 0x00; payload[8]  = 0x00; payload[9]  = 0x00; payload[10] = phase;
  // Entry 3: transition (cmd_id 0x0017, marker 0x80)
  payload[11] = 0x00; payload[12] = 0x17; payload[13] = 0x80;
  payload[14] = (uint8_t)(on_off_transition_ms >> 24);
  payload[15] = (uint8_t)(on_off_transition_ms >> 16);
  payload[16] = (uint8_t)(on_off_transition_ms >> 8);
  payload[17] = (uint8_t)(on_off_transition_ms);
  relay_send_packet(RELAY_TYPE, RELAY_CMD_SET, payload, sizeof(payload));
#else
  relay_send_set(gang, state);
#endif
}

h2_relay_state_t h2_relay_get(uint8_t gang)
{
  if (gang >= h2_active_gang_count) {
    return H2_RELAY_OFF;
  }
  return relay_state[gang];
}

h2_relay_state_t h2_relay_toggle(uint8_t gang)
{
  if (gang >= h2_active_gang_count) {
    return H2_RELAY_OFF;
  }
  h2_relay_state_t new_state = (relay_state[gang] == H2_RELAY_ON) ? H2_RELAY_OFF : H2_RELAY_ON;
  h2_relay_set(gang, new_state);
  return relay_state[gang];
}