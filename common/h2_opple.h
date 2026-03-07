#ifndef H2_OPPLE_H
#define H2_OPPLE_H

#include <stdint.h>
#include <stdbool.h>
#include "h2_button.h"

// Aqara manufacturer code and cluster ID
#define AQARA_MFG_CODE       0x115F
#define OPPLE_CLUSTER_ID     0xFCC0

// Attribute IDs — matching real Aqara H2 (WS-K07E/K08E)
#define OPPLE_ATTR_MODE              0x0009  // event(1) vs command(0) mode
#define OPPLE_ATTR_SWITCH_TYPE       0x000A  // 1=toggle, 2=momentary, 3=none
#define OPPLE_ATTR_FLIP_INDICATOR    0x00F0  // coupled: invert LED; independent: LED on
#define OPPLE_ATTR_OPERATION_MODE    0x0200  // 0=decoupled, 1=control_relay
#define OPPLE_ATTR_LED_INDICATOR     0x0203  // 0=LED coupled to relay, 1=LED independent
#define OPPLE_ATTR_LOCK_RELAY        0x0285  // LED color: 0=blue, 1=red
#define OPPLE_ATTR_MULTI_CLICK       0x0286  // 1=disabled, 2=enabled
#define OPPLE_ATTR_POWER_ON_BEHAVIOR 0x0517  // 0=on, 1=previous, 2=off, 3=inverted

// Operation mode values (note: inverted from typical expectation)
#define OPPLE_OP_MODE_DECOUPLED     0
#define OPPLE_OP_MODE_CONTROL_RELAY 1

// Initialize opple module: load NVM3 values, apply settings.
// Must be called after h2_led_init().
void h2_opple_init(void);

// Handle an incoming ZCL message on cluster 0xFCC0.
// Returns true if message was consumed, false to pass to framework.
bool h2_opple_handle_message(uint8_t endpoint,
                             uint16_t source_node,
                             uint8_t *message,
                             uint16_t msg_len);

// Report a button action via genMultistateInput presentValue attribute report.
// action values: 0=hold, 1=single, 2=double, 255=release
void h2_opple_report_action(uint8_t endpoint, uint8_t action_value);

// Get the current operation mode for a gang.
// Returns OPPLE_OP_MODE_CONTROL_RELAY or OPPLE_OP_MODE_DECOUPLED.
uint8_t h2_opple_get_op_mode(uint8_t gang);

// Persist relay state for power_on_behavior "previous" mode.
// Call whenever relay state changes.
void h2_opple_save_relay_state(uint8_t gang, bool relay_on);

// Apply startup relay state per power_on_behavior + saved previous state.
// Call on NETWORK_UP.
void h2_opple_apply_power_on_behavior(uint8_t gang);

// Re-apply LED settings from current opple config for all gangs.
void h2_opple_init_leds(void);

#endif // H2_OPPLE_H
