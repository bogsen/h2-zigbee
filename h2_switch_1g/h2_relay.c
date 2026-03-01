#include "h2_relay.h"

// TODO: Replace with actual UART protocol when reverse-engineered.
// For now, relay state is stored in RAM only.

#ifndef H2_SWITCH_GANG_COUNT
#define H2_SWITCH_GANG_COUNT 1
#endif

static h2_relay_state_t relay_state[H2_SWITCH_GANG_COUNT];

void h2_relay_init(void)
{
  for (uint8_t i = 0; i < H2_SWITCH_GANG_COUNT; i++) {
    relay_state[i] = H2_RELAY_OFF;
  }
  // TODO: Initialize UART to secondary relay module
}

void h2_relay_set(uint8_t gang, h2_relay_state_t state)
{
  if (gang >= H2_SWITCH_GANG_COUNT) {
    return;
  }
  relay_state[gang] = state;
  // TODO: Send UART command to secondary module
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
  // TODO: Send UART command to secondary module
  return relay_state[gang];
}