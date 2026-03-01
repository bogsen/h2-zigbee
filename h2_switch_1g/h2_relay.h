#ifndef H2_RELAY_H
#define H2_RELAY_H

#include <stdint.h>
#include <stdbool.h>

typedef enum {
  H2_RELAY_OFF = 0,
  H2_RELAY_ON  = 1
} h2_relay_state_t;

// Initialize relay hardware (UART to secondary module)
void h2_relay_init(void);

// Set relay state for a gang (0-indexed)
void h2_relay_set(uint8_t gang, h2_relay_state_t state);

// Get current relay state for a gang (0-indexed)
h2_relay_state_t h2_relay_get(uint8_t gang);

// Toggle relay state for a gang, returns new state
h2_relay_state_t h2_relay_toggle(uint8_t gang);

#endif // H2_RELAY_H