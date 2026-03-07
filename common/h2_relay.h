#ifndef H2_RELAY_H
#define H2_RELAY_H

#include <stdint.h>
#include <stdbool.h>
#include "h2_variant.h"

typedef enum {
  H2_RELAY_OFF = 0,
  H2_RELAY_ON  = 1
} h2_relay_state_t;

// RX callback: called from main loop with parsed command entry fields
// cmd_id: combined command (e.g. 0x0504=power, 0x0601=voltage)
// data_type: entry data type (0x55=float/value)
// data: pointer to 8 data bytes (float at offset 0)
// data_len: always 8
typedef void (*h2_relay_rx_callback_t)(uint16_t cmd_id, uint8_t data_type,
                                       const uint8_t *data, uint8_t data_len);

// Initialize relay hardware + start UART RX
void h2_relay_init(void);

// Set relay state for a gang (0-indexed)
void h2_relay_set(uint8_t gang, h2_relay_state_t state);

// Get current relay state for a gang (0-indexed)
h2_relay_state_t h2_relay_get(uint8_t gang);

// Toggle relay state for a gang, returns new state
h2_relay_state_t h2_relay_toggle(uint8_t gang);

// Register callback for received UART commands
void h2_relay_set_rx_callback(h2_relay_rx_callback_t cb);

// Process pending RX packet (call from main loop, e.g. every 10ms tick)
void h2_relay_process_rx(void);

// Send init handshake to relay module (status + relay state queries)
void h2_relay_send_init(void);

// Send measurement query for a gang (0-indexed)
void h2_relay_send_measurement_query(uint8_t gang);

#if H2_HAS_DIMMER
// Set dimmer brightness level (1-254) for a gang — applies gamma curve and sends via UART
void h2_relay_set_level(uint8_t gang, uint8_t level);

// Get current dimmer brightness level (1-254) for a gang
uint8_t h2_relay_get_level(uint8_t gang);
#endif

#endif // H2_RELAY_H