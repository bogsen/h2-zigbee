#include "app/framework/include/af.h"

#ifdef SL_COMPONENT_CATALOG_PRESENT
#include "sl_component_catalog.h"
#endif

#include "network-steering.h"
#include "find-and-bind-initiator.h"
#include "find-and-bind-target.h"

#if defined(SL_CATALOG_SIMPLE_BUTTON_PRESENT)
#include "sl_simple_button.h"
#include "sl_simple_button_instances.h"
#endif

#include "nvm3_default.h"
#include "btl_interface.h"
#include "slot-manager.h"
#include "h2_relay.h"
#include "h2_button.h"
#include "h2_led.h"
#include "h2_opple.h"
#include "h2_power.h"
#include "h2_temperature.h"
#include "sl_sleeptimer.h"
#include "em_gpio.h"
#include "pin_config.h"

#define H2_FIRMWARE_VERSION  0x00000072  // increment for each OTA release
#define H2_IMAGE_TYPE_ID     0xEDCF      // unique per product variant

#define CONTROL_ENDPOINT          11
#define BUTTON_TICK_MS            10
#define LED_TICK_MS               20
#define FINDING_AND_BINDING_DELAY_MS 3000
#define COMMISSIONING_HOLD_MS     5000
#define FACTORY_RESET_HOLD_MS     10000

#define NVM3_KEY_GANG_COUNT       0x00F0

// Runtime gang count (1 or 2), set at boot before module inits
uint8_t h2_active_gang_count = 1;

// Per-gang runtime state
typedef struct {
  uint8_t endpoint;
  uint8_t control_endpoint;
} gang_state_t;

static gang_state_t gangs[2] = {
  { .endpoint = 1, .control_endpoint = CONTROL_ENDPOINT },
  { .endpoint = 2, .control_endpoint = CONTROL_ENDPOINT + 1 },
};

//--------------------------------------------------
// Bound-light report routing

static uint16_t binding_group_id(const sl_zigbee_binding_table_entry_t *entry)
{
  return (uint16_t)entry->identifier[0]
         | ((uint16_t)entry->identifier[1] << 8);
}

static bool has_matching_multicast_binding(uint8_t local_ep,
                                           sl_zigbee_af_cluster_id_t cluster_id,
                                           uint16_t group_id)
{
  uint8_t table_size = sl_zigbee_get_binding_table_size();
  for (uint8_t i = 0; i < table_size; i++) {
    sl_zigbee_binding_table_entry_t entry;
    if (sl_zigbee_get_binding(i, &entry) != SL_STATUS_OK) {
      continue;
    }
    if (entry.type != SL_ZIGBEE_MULTICAST_BINDING
        || entry.local != local_ep
        || entry.clusterId != cluster_id) {
      continue;
    }
    if (binding_group_id(&entry) == group_id) {
      return true;
    }
  }
  return false;
}

static bool report_applies_to_endpoint(const sl_zigbee_af_cluster_command_t *cmd,
                                       uint8_t local_ep,
                                       sl_zigbee_af_cluster_id_t cluster_id)
{
  if (cmd == NULL || cmd->apsFrame == NULL) {
    return false;
  }
  uint8_t dst_ep = cmd->apsFrame->destinationEndpoint;
  if (dst_ep == local_ep) {
    return true;
  }
  if (dst_ep == 0xFF) {
    return has_matching_multicast_binding(local_ep, cluster_id,
                                          cmd->apsFrame->groupId);
  }
  return false;
}

static bool parse_report_attribute_u8(const uint8_t *message,
                                      uint16_t msg_len,
                                      sl_zigbee_af_attribute_id_t expected_attr_id,
                                      uint8_t *value)
{
  if (message == NULL || value == NULL || msg_len < 3) {
    return false;
  }

  uint8_t frame_control = message[0];
  if ((frame_control & ZCL_FRAME_CONTROL_FRAME_TYPE_MASK) != ZCL_GLOBAL_COMMAND) {
    return false;
  }

  uint16_t sequence_index = 1;
  if ((frame_control & ZCL_MANUFACTURER_SPECIFIC_MASK) != 0) {
    sequence_index += 2;
  }
  uint16_t command_index = sequence_index + 1;
  uint16_t payload_index = command_index + 1;
  if (payload_index + 4 > msg_len
      || message[command_index] != ZCL_REPORT_ATTRIBUTES_COMMAND_ID) {
    return false;
  }

  uint16_t attr_id = message[payload_index]
                     | ((uint16_t)message[payload_index + 1] << 8);
  if (attr_id != expected_attr_id) {
    return false;
  }

  *value = message[payload_index + 3];
  return true;
}

static bool commissioning = false;

// Factory reset state
static bool factory_reset_active = false;
static uint16_t factory_reset_timer_ms = 0;

// Events
static sl_zigbee_af_event_t button_tick_event;
static sl_zigbee_af_event_t led_tick_event;
static sl_zigbee_af_event_t finding_and_binding_event;
static sl_zigbee_af_event_t commissioning_event;
static sl_zigbee_af_event_t factory_reset_event;
static sl_zigbee_af_event_t light_sync_event;
static sl_zigbee_af_event_t power_on_event;

//--------------------------------------------------
// Forward declarations
static void button_tick_handler(sl_zigbee_af_event_t *event);
static void led_tick_handler(sl_zigbee_af_event_t *event);
static void finding_and_binding_handler(sl_zigbee_af_event_t *event);
static void commissioning_event_handler(sl_zigbee_af_event_t *event);
static void factory_reset_event_handler(sl_zigbee_af_event_t *event);
static void light_sync_handler(sl_zigbee_af_event_t *event);
static void on_button_action(uint8_t gang, h2_button_side_t side, h2_button_action_t action);

//--------------------------------------------------
// NVM3 helpers (local to app.c)

static uint8_t nvm3_read_u8(nvm3_ObjectKey_t key, uint8_t default_val)
{
  uint8_t val;
  Ecode_t err = nvm3_readData(nvm3_defaultHandle, key, &val, sizeof(val));
  return (err == ECODE_NVM3_OK) ? val : default_val;
}

static void nvm3_write_u8(nvm3_ObjectKey_t key, uint8_t val)
{
  nvm3_writeData(nvm3_defaultHandle, key, &val, sizeof(val));
}

//--------------------------------------------------
// Model identifier: external attribute storage (set in ZAP as "External")
// Returns model string based on detected gang count.

static const uint8_t model_1g[] = "\x12" "lumi.switch.agl009";
static const uint8_t model_2g[] = "\x12" "lumi.switch.agl010";

sl_zigbee_af_status_t sl_zigbee_af_external_attribute_read_cb(
    uint8_t endpoint,
    sl_zigbee_af_cluster_id_t clusterId,
    sl_zigbee_af_attribute_metadata_t *attributeMetadata,
    uint16_t manufacturerCode,
    uint8_t *buffer,
    uint16_t maxReadLength)
{
  if (clusterId == ZCL_BASIC_CLUSTER_ID
      && attributeMetadata->attributeId == ZCL_MODEL_IDENTIFIER_ATTRIBUTE_ID) {
    const uint8_t *model = (h2_active_gang_count >= 2) ? model_2g : model_1g;
    uint8_t total = 1 + model[0];
    if (total > maxReadLength) {
      return SL_ZIGBEE_ZCL_STATUS_INSUFFICIENT_SPACE;
    }
    memcpy(buffer, model, total);
    return SL_ZIGBEE_ZCL_STATUS_SUCCESS;
  }
  return SL_ZIGBEE_ZCL_STATUS_FAILURE;
}

sl_zigbee_af_status_t sl_zigbee_af_external_attribute_write_cb(
    uint8_t endpoint,
    sl_zigbee_af_cluster_id_t clusterId,
    sl_zigbee_af_attribute_metadata_t *attributeMetadata,
    uint16_t manufacturerCode,
    uint8_t *buffer)
{
  if (clusterId == ZCL_BASIC_CLUSTER_ID
      && attributeMetadata->attributeId == ZCL_MODEL_IDENTIFIER_ATTRIBUTE_ID) {
    return SL_ZIGBEE_ZCL_STATUS_SUCCESS;
  }
  return SL_ZIGBEE_ZCL_STATUS_FAILURE;
}

//--------------------------------------------------
// Relay <-> On/Off cluster synchronization

static void sync_relay_to_cluster(uint8_t gang)
{
  h2_relay_state_t state = h2_relay_get(gang);
  uint8_t on_off_val = (state == H2_RELAY_ON) ? 1 : 0;
  sl_zigbee_af_write_server_attribute(
    gangs[gang].endpoint,
    ZCL_ON_OFF_CLUSTER_ID,
    ZCL_ON_OFF_ATTRIBUTE_ID,
    &on_off_val,
    ZCL_BOOLEAN_ATTRIBUTE_TYPE);
  h2_led_update_relay_state(gang, state == H2_RELAY_ON);
  h2_opple_save_relay_state(gang, state == H2_RELAY_ON);
}

//--------------------------------------------------
// Restore after ZCL initialization, independently of network availability.

static void power_on_handler(sl_zigbee_af_event_t *event)
{
  sl_zigbee_af_event_set_inactive(event);
  for (uint8_t g = 0; g < h2_active_gang_count; g++) {
    h2_opple_apply_power_on_behavior(g);
    sync_relay_to_cluster(g);
  }
}

//--------------------------------------------------
// Coupled mode: button controls local relay

static void handle_coupled(uint8_t gang, h2_button_side_t side, h2_button_action_t action)
{
  if (action != H2_BTN_ACTION_PRESS) {
    return; // relay already toggled on PRESS
  }

  if (side == H2_BTN_SIDE_UP) {
    h2_relay_set(gang, H2_RELAY_OFF);
  } else {
    h2_relay_set(gang, H2_RELAY_ON);
  }
  sync_relay_to_cluster(gang);
}

//--------------------------------------------------
// Decoupled mode: button sends ZCL commands to bindings

// Declared in framework util/common.c
extern uint8_t appResponseData[];

#define MOVE_RATE_SLOW  60   // units/sec for hold-to-dim
#define MOVE_RATE_FAST  254  // units/sec for quick tap toggle

static void send_to_bindings(void)
{
  // Disable default response to prevent response storm from every recipient
  appResponseData[0] |= ZCL_DISABLE_DEFAULT_RESPONSE_MASK;

  // Disable APS retry so failed deliveries don't block the TX queue
  sl_zigbee_af_get_command_aps_frame()->options &= ~SL_ZIGBEE_APS_OPTION_RETRY;

  sl_zigbee_af_send_command_multicast_to_bindings();
  sl_zigbee_af_send_command_unicast_to_bindings();
}

// Helper: fill command, set source endpoint, send
static void send_cmd(uint8_t ep)
{
  sl_zigbee_af_get_command_aps_frame()->sourceEndpoint = ep;
  send_to_bindings();
}

static void handle_decoupled(uint8_t gang, h2_button_side_t side, h2_button_action_t action)
{
  uint8_t ep = gangs[gang].control_endpoint;

  switch (action) {
    case H2_BTN_ACTION_PRESS:
      // Immediate move on press — near-zero latency dimming
      if (side == H2_BTN_SIDE_DOWN) {
        // move_with_on_off so light turns on if off
        sl_zigbee_af_fill_command_level_control_cluster_move_with_on_off(0, MOVE_RATE_SLOW);
        send_cmd(ep);
        // Also send On for non-dimmable lights bound via On/Off cluster
        sl_zigbee_af_fill_command_on_off_cluster_on();
        send_cmd(ep);
        // Show dimmed (both LEDs) while brightness is ramping up
        h2_led_update_light_level(gang, 127);
      } else {
        // move (without on_off) so light doesn't turn off at level 0 during dim
        sl_zigbee_af_fill_command_level_control_cluster_move(1, MOVE_RATE_SLOW, 0, 0);
        send_cmd(ep);
        // Show dimmed (both LEDs) while brightness is ramping down
        if (h2_led_get_light_state(gang)) {
          h2_led_update_light_level(gang, 127);
        }
      }
      break;

    case H2_BTN_ACTION_SINGLE:
      // Quick tap: jump to target level. Implicitly stops any running move.
      // Single command instead of stop + move = less radio congestion.
      if (side == H2_BTN_SIDE_DOWN) {
        sl_zigbee_af_fill_command_level_control_cluster_move_to_level_with_on_off(254, 5);
        send_cmd(ep);
        // Also send On for non-dimmable lights bound via On/Off cluster
        sl_zigbee_af_fill_command_on_off_cluster_on();
        send_cmd(ep);
        h2_led_update_light_state(gang, true);
      } else {
        sl_zigbee_af_fill_command_level_control_cluster_move_to_level_with_on_off(0, 5);
        send_cmd(ep);
        // Also send Off for non-dimmable lights bound via On/Off cluster
        sl_zigbee_af_fill_command_on_off_cluster_off();
        send_cmd(ep);
        h2_led_update_light_state(gang, false);
      }
      break;

    case H2_BTN_ACTION_LONG:
      // No-op: slow move from PRESS is already running
      break;

    case H2_BTN_ACTION_RELEASE:
      // Freeze light at current level
      sl_zigbee_af_fill_command_level_control_cluster_stop_with_on_off();
      send_cmd(ep);
      // Light is at an intermediate level — show both LEDs
      if (side == H2_BTN_SIDE_DOWN || h2_led_get_light_state(gang)) {
        h2_led_update_light_level(gang, 127);
      }
      break;

    case H2_BTN_ACTION_DOUBLE:
      // Cancel any running move
      sl_zigbee_af_fill_command_level_control_cluster_stop_with_on_off();
      send_cmd(ep);
      // Light may be at an intermediate level
      if (h2_led_get_light_state(gang)) {
        h2_led_update_light_level(gang, 127);
      }
      break;
  }
}

//--------------------------------------------------
// Button callback (called from h2_button state machine via tick)

static void on_button_action(uint8_t gang, h2_button_side_t side, h2_button_action_t action)
{
  if (gang >= h2_active_gang_count) return;

  sl_zigbee_app_debug_println("Button: gang=%d side=%d action=%d", gang, side, action);

  // PRESS: low-latency immediate action for both modes
  if (action == H2_BTN_ACTION_PRESS) {
    if (h2_opple_get_op_mode(gang) == OPPLE_OP_MODE_CONTROL_RELAY) {
      handle_coupled(gang, side, action);
    } else if (sl_zigbee_af_network_state() == SL_ZIGBEE_JOINED_NETWORK) {
      handle_decoupled(gang, side, action);
    }
    return;
  }

  // Subsequent actions (SINGLE/DOUBLE/LONG/RELEASE) — no relay action in coupled mode
  // (already fired on PRESS), but still report and handle decoupled

  if (sl_zigbee_af_network_state() != SL_ZIGBEE_JOINED_NETWORK) {
    if (action == H2_BTN_ACTION_LONG) {
      sl_zigbee_af_event_set_active(&commissioning_event);
    }
    return;
  }

  // Map button action to Aqara action values for reporting
  // 0=hold, 1=single, 2=double, 255=release
  static const uint8_t action_map[] = {
    [H2_BTN_ACTION_RELEASE] = 255,
    [H2_BTN_ACTION_SINGLE]  = 1,
    [H2_BTN_ACTION_DOUBLE]  = 2,
    [H2_BTN_ACTION_LONG]    = 0,
  };

  // Map (gang, side) to the endpoint for button action reporting
  // 1-gang: up=EP1, down=EP4.  2-gang: gang0 up=EP1 down=EP4, gang1 up=EP2 down=EP5
  uint8_t report_ep = gangs[gang].endpoint
                      + ((side == H2_BTN_SIDE_DOWN) ? 3 : 0);

  // Decoupled mode: send ZCL commands to bindings (no local relay control)
  if (h2_opple_get_op_mode(gang) != OPPLE_OP_MODE_CONTROL_RELAY) {
    handle_decoupled(gang, side, action);
  }

  // Report button event to coordinator via genMultistateInput
  h2_opple_report_action(report_ep, action_map[action]);
}

//--------------------------------------------------
// Periodic tick events

static void button_tick_handler(sl_zigbee_af_event_t *event)
{
  (void)event;
  h2_relay_process_rx();
  h2_button_tick();

  // Detect both buttons held for factory reset
#if SL_SIMPLE_BUTTON_COUNT > 1
  bool btn0_pressed = (sl_button_get_state(SL_SIMPLE_BUTTON_INSTANCE(0))
                       == SL_SIMPLE_BUTTON_PRESSED);
  bool btn1_pressed = (sl_button_get_state(SL_SIMPLE_BUTTON_INSTANCE(1))
                       == SL_SIMPLE_BUTTON_PRESSED);

  if (btn0_pressed && btn1_pressed) {
    factory_reset_timer_ms += BUTTON_TICK_MS;
    if (!factory_reset_active && factory_reset_timer_ms >= 1000) {
      factory_reset_active = true;
      h2_led_set_mode(0, H2_LED_MODE_MANUAL);
      h2_led_set_manual(0, 0, 100);  // red = warning
    }
    if (factory_reset_timer_ms >= FACTORY_RESET_HOLD_MS) {
      sl_zigbee_af_event_set_active(&factory_reset_event);
      factory_reset_timer_ms = 0;
      factory_reset_active = false;
    }
  } else {
    if (factory_reset_active) {
      factory_reset_active = false;
      h2_opple_init_leds();
    }
    factory_reset_timer_ms = 0;
  }
#endif

  sl_zigbee_af_event_set_delay_ms(&button_tick_event, BUTTON_TICK_MS);
}

static void led_tick_handler(sl_zigbee_af_event_t *event)
{
  (void)event;
  h2_led_tick();
  sl_zigbee_af_event_set_delay_ms(&led_tick_event, LED_TICK_MS);
}

//--------------------------------------------------
// Commissioning

static void commissioning_event_handler(sl_zigbee_af_event_t *event)
{
  (void)event;

  if (sl_zigbee_af_network_state() != SL_ZIGBEE_JOINED_NETWORK) {
    sl_status_t status = sl_zigbee_af_network_steering_start();
    sl_zigbee_app_debug_println("Join network start: 0x%02X", status);
    commissioning = true;
  }
}

static void factory_reset_event_handler(sl_zigbee_af_event_t *event)
{
  (void)event;
  sl_zigbee_app_debug_println("*** FACTORY RESET ***");

  // Visual feedback: blue flash = reset complete
  h2_led_set_mode(0, H2_LED_MODE_MANUAL);
  h2_led_set_manual(0, 100, 100);

  sl_zigbee_clear_binding_table();
  nvm3_eraseAll(nvm3_defaultHandle);
  sl_zigbee_leave_network(SL_ZIGBEE_LEAVE_NWK_WITH_NO_OPTION);
  halReboot();
}

static void finding_and_binding_handler(sl_zigbee_af_event_t *event)
{
  (void)event;
  sl_status_t status = sl_zigbee_af_find_and_bind_initiator_start(CONTROL_ENDPOINT);
  sl_zigbee_app_debug_println("Find and bind initiator start: 0x%02X", status);
}

//--------------------------------------------------
// Pre-attribute-change callback
// Rejects self-binding loopback: decoupled button -> binding -> self.
// Binding-based On/Off from other devices is already consumed by
// pre_message_received_cb (ep 0xFF check).

sl_zigbee_af_status_t sl_zigbee_af_pre_attribute_change_cb(
    uint8_t endpoint,
    sl_zigbee_af_cluster_id_t clusterId,
    sl_zigbee_af_attribute_id_t attributeId,
    uint8_t mask,
    uint16_t manufacturerCode,
    uint8_t type,
    uint8_t size,
    uint8_t *value)
{
  (void)mask;
  (void)manufacturerCode;
  (void)type;
  (void)size;
  (void)value;

  if (clusterId == ZCL_ON_OFF_CLUSTER_ID
      && attributeId == ZCL_ON_OFF_ATTRIBUTE_ID) {
    const sl_zigbee_af_cluster_command_t *cmd = sl_zigbee_af_current_command();
    if (cmd != NULL && cmd->source == sl_zigbee_af_get_node_id()) {
      sl_zigbee_app_debug_println("Rejected self-loopback On/Off on EP%d", endpoint);
      return SL_ZIGBEE_ZCL_STATUS_FAILURE;
    }
  }

  return SL_ZIGBEE_ZCL_STATUS_SUCCESS;
}

//--------------------------------------------------
// Post-attribute-change callback
// Called after ANY attribute value changes (including from On/Off server plugin)

void sl_zigbee_af_post_attribute_change_cb(uint8_t endpoint,
                                            sl_zigbee_af_cluster_id_t clusterId,
                                            sl_zigbee_af_attribute_id_t attributeId,
                                            uint8_t mask,
                                            uint16_t manufacturerCode,
                                            uint8_t type,
                                            uint8_t size,
                                            uint8_t *value)
{
  (void)manufacturerCode;
  (void)type;
  (void)size;

  if (clusterId != ZCL_ON_OFF_CLUSTER_ID
      || attributeId != ZCL_ON_OFF_ATTRIBUTE_ID
      || mask != CLUSTER_MASK_SERVER) {
    return;
  }

  uint8_t on_off_val = *value;

  // Find which gang this endpoint belongs to
  for (uint8_t g = 0; g < h2_active_gang_count; g++) {
    if (gangs[g].endpoint == endpoint) {
      h2_relay_set(g, on_off_val ? H2_RELAY_ON : H2_RELAY_OFF);
      h2_led_update_relay_state(g, on_off_val != 0);
      h2_opple_save_relay_state(g, on_off_val != 0);
      sl_zigbee_app_debug_println("Relay %d: %s", g, on_off_val ? "ON" : "OFF");
      break;
    }
  }
}

//--------------------------------------------------
// Report Attributes callback
// Receives On/Off attribute reports from bound lights so the LED
// can track their actual state (e.g. when controlled by another device).

bool sl_zigbee_af_report_attributes_cb(sl_zigbee_af_cluster_id_t clusterId,
                                        uint8_t *buffer,
                                        uint16_t bufLen)
{
  if (bufLen < 4) {
    return false;
  }

  // Report Attributes payload: attr_id (2B LE) + type (1B) + value (1B)
  uint16_t attr_id = buffer[0] | ((uint16_t)buffer[1] << 8);

  const sl_zigbee_af_cluster_command_t *cmd = sl_zigbee_af_current_command();
  if (cmd == NULL || cmd->apsFrame == NULL) {
    return false;
  }

  if (clusterId == ZCL_ON_OFF_CLUSTER_ID
      && attr_id == ZCL_ON_OFF_ATTRIBUTE_ID) {
    uint8_t on_off_val = buffer[3];
    for (uint8_t g = 0; g < h2_active_gang_count; g++) {
      uint8_t endpoint = gangs[g].control_endpoint;
      if (report_applies_to_endpoint(cmd, endpoint, clusterId)) {
        h2_led_update_light_state(g, on_off_val != 0);
        if (cmd->apsFrame->destinationEndpoint == 0xFF) {
          sl_zigbee_app_debug_println("Light report EP%d group 0x%04X: %s",
                                       endpoint,
                                       cmd->apsFrame->groupId,
                                       on_off_val ? "ON" : "OFF");
        } else {
          sl_zigbee_app_debug_println("Light report EP%d: %s", endpoint,
                                       on_off_val ? "ON" : "OFF");
        }
      }
    }
  } else if (clusterId == ZCL_LEVEL_CONTROL_CLUSTER_ID
             && attr_id == ZCL_CURRENT_LEVEL_ATTRIBUTE_ID) {
    uint8_t level = buffer[3];
    for (uint8_t g = 0; g < h2_active_gang_count; g++) {
      uint8_t endpoint = gangs[g].control_endpoint;
      if (report_applies_to_endpoint(cmd, endpoint, clusterId)) {
        h2_led_update_light_level(g, level);
        if (cmd->apsFrame->destinationEndpoint == 0xFF) {
          sl_zigbee_app_debug_println("Level report EP%d group 0x%04X: %d",
                                       endpoint,
                                       cmd->apsFrame->groupId,
                                       level);
        } else {
          sl_zigbee_app_debug_println("Level report EP%d: %d", endpoint, level);
        }
      }
    }
  }

  return false;  // let framework continue processing
}

//--------------------------------------------------
// Read Attributes Response callback
// Handles responses to our boot-time light state query.

bool sl_zigbee_af_read_attributes_response_cb(sl_zigbee_af_cluster_id_t clusterId,
                                               uint8_t *buffer,
                                               uint16_t bufLen)
{
  if (bufLen < 5) {
    return false;
  }

  // Read Attributes Response: attr_id (2B LE) + status (1B) + type (1B) + value (1B)
  uint16_t attr_id = buffer[0] | ((uint16_t)buffer[1] << 8);
  if (buffer[2] != 0) {
    return false;  // read failed
  }

  const sl_zigbee_af_cluster_command_t *cmd = sl_zigbee_af_current_command();
  if (cmd == NULL || cmd->apsFrame == NULL) {
    return false;
  }
  uint8_t endpoint = cmd->apsFrame->destinationEndpoint;

  if (clusterId == ZCL_ON_OFF_CLUSTER_ID
      && attr_id == ZCL_ON_OFF_ATTRIBUTE_ID) {
    uint8_t on_off_val = buffer[4];
    for (uint8_t g = 0; g < h2_active_gang_count; g++) {
      if (gangs[g].control_endpoint == endpoint) {
        h2_led_update_light_state(g, on_off_val != 0);
        sl_zigbee_app_debug_println("Light sync EP%d: %s", endpoint,
                                     on_off_val ? "ON" : "OFF");
        break;
      }
    }
  } else if (clusterId == ZCL_LEVEL_CONTROL_CLUSTER_ID
             && attr_id == ZCL_CURRENT_LEVEL_ATTRIBUTE_ID) {
    uint8_t level = buffer[4];
    for (uint8_t g = 0; g < h2_active_gang_count; g++) {
      if (gangs[g].control_endpoint == endpoint) {
        h2_led_update_light_level(g, level);
        sl_zigbee_app_debug_println("Level sync EP%d: %d", endpoint, level);
        break;
      }
    }
  }

  return false;
}

//--------------------------------------------------
// Light state sync: read On/Off from bound lights on boot

static void light_sync_handler(sl_zigbee_af_event_t *event)
{
  (void)event;
  for (uint8_t g = 0; g < h2_active_gang_count; g++) {
    if (h2_opple_get_op_mode(g) != OPPLE_OP_MODE_CONTROL_RELAY) {
      uint8_t ep = gangs[g].control_endpoint;
      uint8_t attr_ids[] = {
        LOW_BYTE(ZCL_ON_OFF_ATTRIBUTE_ID),
        HIGH_BYTE(ZCL_ON_OFF_ATTRIBUTE_ID)
      };
      // Query On/Off state
      sl_zigbee_af_fill_command_global_client_to_server_read_attributes(
          ZCL_ON_OFF_CLUSTER_ID, attr_ids, sizeof(attr_ids));
      sl_zigbee_af_get_command_aps_frame()->sourceEndpoint = ep;
      sl_zigbee_af_send_command_multicast_to_bindings();
      sl_zigbee_af_send_command_unicast_to_bindings();
      // Query CurrentLevel for dimmed state detection
      uint8_t level_attr_ids[] = {
        LOW_BYTE(ZCL_CURRENT_LEVEL_ATTRIBUTE_ID),
        HIGH_BYTE(ZCL_CURRENT_LEVEL_ATTRIBUTE_ID)
      };
      sl_zigbee_af_fill_command_global_client_to_server_read_attributes(
          ZCL_LEVEL_CONTROL_CLUSTER_ID, level_attr_ids, sizeof(level_attr_ids));
      sl_zigbee_af_get_command_aps_frame()->sourceEndpoint = ep;
      sl_zigbee_af_send_command_multicast_to_bindings();
      sl_zigbee_af_send_command_unicast_to_bindings();
    }
  }
}

//--------------------------------------------------
// Zigbee framework callbacks

void sl_zigbee_af_main_init_cb(void)
{
  // Erase storage slot 0 so stale OTA images don't get re-applied on boot
  bootloader_eraseStorageSlot(0);

  // Initialize relay UART + power RX callback (needed for gang detection)
  h2_relay_init();
  h2_power_init();

  // --- Gang count detection via relay UART ---
  // Read NVM3 fallback first
  uint8_t fallback_gangs = nvm3_read_u8(NVM3_KEY_GANG_COUNT, 1);

  // Reset relay module to trigger announce packet (contains gang count in cmd 0x0000)
  GPIO_PinOutClear(WALL7_PORT, WALL7_PIN);
  sl_sleeptimer_delay_millisecond(100);
  GPIO_PinOutSet(WALL7_PORT, WALL7_PIN);

  // Wait for announce packet (relay boots in ~1-2s)
  for (int i = 0; i < 200 && h2_power_get_detected_gangs() == 0; i++) {
    h2_relay_process_rx();
    sl_sleeptimer_delay_millisecond(10);
  }

  uint8_t detected = h2_power_get_detected_gangs();
  if (detected > 0) {
    h2_active_gang_count = (detected > 2) ? 2 : detected;
  } else {
    h2_active_gang_count = (fallback_gangs > 2) ? 2 : (fallback_gangs < 1) ? 1 : fallback_gangs;
  }
  nvm3_write_u8(NVM3_KEY_GANG_COUNT, h2_active_gang_count);
  sl_zigbee_app_debug_println("Gang count: %d (detected=%d, fallback=%d)",
                               h2_active_gang_count, detected, fallback_gangs);

  // Initialize remaining modules (now that h2_active_gang_count is set)
  h2_button_init(on_button_action);
  h2_led_init();
  h2_opple_init();
  h2_temperature_init();

  // Send init handshake to relay module (status + relay state queries)
  h2_relay_send_init();

  // Initialize events
  sl_zigbee_af_event_init(&button_tick_event, button_tick_handler);
  sl_zigbee_af_event_init(&led_tick_event, led_tick_handler);
  sl_zigbee_af_event_init(&finding_and_binding_event, finding_and_binding_handler);
  sl_zigbee_af_event_init(&commissioning_event, commissioning_event_handler);
  sl_zigbee_af_event_init(&factory_reset_event, factory_reset_event_handler);
  sl_zigbee_af_event_init(&light_sync_event, light_sync_handler);
  sl_zigbee_af_event_init(&power_on_event, power_on_handler);
  sl_zigbee_af_event_set_active(&power_on_event);

  // Reduce APS ACK timeout from default 1600ms to 500ms.
  // Default assumes 30 hops; a home network is 1-2 hops.
  // Long timeouts block TX queue slots during retries.
  sl_zigbee_set_aps_ack_timeout_ms(500);

  // Start periodic ticks
  sl_zigbee_af_event_set_delay_ms(&button_tick_event, BUTTON_TICK_MS);
  sl_zigbee_af_event_set_delay_ms(&led_tick_event, LED_TICK_MS);
}

void sl_zigbee_af_stack_status_cb(sl_status_t status)
{
  if (status == SL_STATUS_NETWORK_UP) {
    sl_zigbee_app_debug_println("Network UP");
    // Start OTA client immediately (no random delay)
    sl_zigbee_af_ota_client_start_cb();
    // Start measurement polling
    h2_power_start();
    h2_temperature_start();
    // Sync LED to bound light state after network settles
    sl_zigbee_af_event_set_delay_ms(&light_sync_event, 2000);
  } else if (status == SL_STATUS_NETWORK_DOWN) {
    sl_zigbee_app_debug_println("Network DOWN");
  }
}

void sl_zigbee_af_network_steering_complete_cb(sl_status_t status,
                                               uint8_t totalBeacons,
                                               uint8_t joinAttempts,
                                               uint8_t finalState)
{
  (void)totalBeacons;
  (void)joinAttempts;
  (void)finalState;

  sl_zigbee_app_debug_println("Join network complete: 0x%02X", status);

  if (status != SL_STATUS_OK) {
    commissioning = false;
  } else {
    sl_zigbee_af_event_set_delay_ms(&finding_and_binding_event,
                                    FINDING_AND_BINDING_DELAY_MS);
  }
}

void sl_zigbee_af_find_and_bind_initiator_complete_cb(sl_status_t status)
{
  sl_zigbee_app_debug_println("Find and bind initiator complete: 0x%02X", status);
  commissioning = false;
}

#ifndef SL_CATALOG_ZIGBEE_EZSP_PRESENT
void sl_zigbee_af_radio_needs_calibrating_cb(void)
{
  sl_mac_calibrate_current_channel();
}
#endif

//--------------------------------------------------
// Pre-message-received: intercept messages before framework processing

bool sl_zigbee_af_pre_message_received_cb(sl_zigbee_af_incoming_message_t *msg)
{
  // Handle Opple cluster (0xFCC0)
  if (msg->apsFrame->clusterId == OPPLE_CLUSTER_ID) {
    return h2_opple_handle_message(msg->apsFrame->destinationEndpoint,
                                   msg->source,
                                   msg->message,
                                   msg->msgLen);
  }

  // Filter self-loopback: when server On/Off changes, framework sends
  // Report Attributes to multicast bindings. We receive our own report,
  // which would incorrectly update light_level in decoupled mode.
  if (msg->apsFrame->clusterId == ZCL_ON_OFF_CLUSTER_ID
      && msg->apsFrame->destinationEndpoint == 0xFF
      && msg->source == sl_zigbee_af_get_node_id()) {
    return true;  // consume self-loopback (commands and reports)
  }

  // Route group feedback using control bindings before framework endpoint dispatch.
  if (msg->apsFrame->destinationEndpoint == 0xFF) {
    if (msg->apsFrame->clusterId == ZCL_ON_OFF_CLUSTER_ID) {
      uint8_t on_off_val;
      if (parse_report_attribute_u8(msg->message,
                                    msg->msgLen,
                                    ZCL_ON_OFF_ATTRIBUTE_ID,
                                    &on_off_val)) {
        bool matched = false;
        for (uint8_t g = 0; g < h2_active_gang_count; g++) {
          uint8_t endpoint = gangs[g].control_endpoint;
          if (has_matching_multicast_binding(endpoint,
                                             ZCL_ON_OFF_CLUSTER_ID,
                                             msg->apsFrame->groupId)) {
            h2_led_update_light_state(g, on_off_val != 0);
            sl_zigbee_app_debug_println("Light report EP%d group 0x%04X raw: %s",
                                         endpoint,
                                         msg->apsFrame->groupId,
                                         on_off_val ? "ON" : "OFF");
            matched = true;
          }
        }
        if (!matched) {
          sl_zigbee_app_debug_println("Ignored light report group 0x%04X",
                                       msg->apsFrame->groupId);
        }
        return true;
      }
    } else if (msg->apsFrame->clusterId == ZCL_LEVEL_CONTROL_CLUSTER_ID) {
      uint8_t level;
      if (parse_report_attribute_u8(msg->message,
                                    msg->msgLen,
                                    ZCL_CURRENT_LEVEL_ATTRIBUTE_ID,
                                    &level)) {
        bool matched = false;
        for (uint8_t g = 0; g < h2_active_gang_count; g++) {
          uint8_t endpoint = gangs[g].control_endpoint;
          if (has_matching_multicast_binding(endpoint,
                                             ZCL_LEVEL_CONTROL_CLUSTER_ID,
                                             msg->apsFrame->groupId)) {
            h2_led_update_light_level(g, level);
            sl_zigbee_app_debug_println("Level report EP%d group 0x%04X raw: %d",
                                         endpoint,
                                         msg->apsFrame->groupId,
                                         level);
            matched = true;
          }
        }
        if (!matched) {
          sl_zigbee_app_debug_println("Ignored level report group 0x%04X",
                                       msg->apsFrame->groupId);
        }
        return true;
      }
    }
  }

  // Consume group control in decoupled mode, updating only matching gangs.
  if (msg->apsFrame->clusterId == ZCL_ON_OFF_CLUSTER_ID
      && msg->apsFrame->destinationEndpoint == 0xFF
      && msg->msgLen >= 3
      && (msg->message[0] & 0x03) == 0x01) {  // cluster-specific frame type
    bool decoupled = false;
    for (uint8_t g = 0; g < h2_active_gang_count; g++) {
      bool gang_decoupled = h2_opple_get_op_mode(g) != OPPLE_OP_MODE_CONTROL_RELAY;
      decoupled |= gang_decoupled;
      if ((msg->message[0] & (ZCL_MANUFACTURER_SPECIFIC_MASK
                             | ZCL_FRAME_CONTROL_SERVER_TO_CLIENT)) == 0
          && gang_decoupled
          && has_matching_multicast_binding(gangs[g].control_endpoint,
                                             ZCL_ON_OFF_CLUSTER_ID,
                                             msg->apsFrame->groupId)) {
        // Update light state from the command for immediate LED feedback
        uint8_t cmd_id = msg->message[2];
        if (cmd_id == 0x01) {       // On
          h2_led_update_light_state(g, true);
        } else if (cmd_id == 0x00) { // Off
          h2_led_update_light_state(g, false);
        }
      }
    }
    return decoupled;  // preserve group relay control when all gangs are coupled
  }

  return false;
}

//--------------------------------------------------
// Button ISR -> h2_button state machine

#if defined(SL_CATALOG_SIMPLE_BUTTON_PRESENT)
void sl_button_on_change(const sl_button_t *handle)
{
  bool pressed = (sl_button_get_state(handle) == SL_SIMPLE_BUTTON_PRESSED);

  // Map simple_button instances to button indices
  // btn1 = index 0 (gang 0/1 up), btn2 = index 1 (gang 0/1 down)
  // btn3 = index 2 (gang 0 up, 2-gang only), btn4 = index 3 (gang 0 down, 2-gang only)
  if (SL_SIMPLE_BUTTON_INSTANCE(0) == handle) {
    h2_button_on_change(0, pressed);
  }
#if SL_SIMPLE_BUTTON_COUNT > 1
  else if (SL_SIMPLE_BUTTON_INSTANCE(1) == handle) {
    h2_button_on_change(1, pressed);
  }
#endif
#if SL_SIMPLE_BUTTON_COUNT > 2
  else if (SL_SIMPLE_BUTTON_INSTANCE(2) == handle) {
    h2_button_on_change(2, pressed);
  }
#endif
#if SL_SIMPLE_BUTTON_COUNT > 3
  else if (SL_SIMPLE_BUTTON_INSTANCE(3) == handle) {
    h2_button_on_change(3, pressed);
  }
#endif
}
#endif // SL_CATALOG_SIMPLE_BUTTON_PRESENT

//--------------------------------------------------
// OTA client: bootload callback
// The ota-client-policy component is not used; override the WEAK default
// to use the slot manager API for bootloading.

void sl_zigbee_af_ota_client_bootload_cb(const sl_zigbee_af_ota_image_id_t *id)
{
  (void)id;
  sl_util_af_slot_manager_boot_slot(0);
}

//--------------------------------------------------
// OTA client: report firmware identity to the OTA server

void sl_zigbee_af_ota_client_version_info_cb(sl_zigbee_af_ota_image_id_t *currentImageInfo,
                                              uint16_t *hardwareVersion)
{
  if (currentImageInfo != NULL) {
    memset(currentImageInfo, 0, sizeof(sl_zigbee_af_ota_image_id_t));
    currentImageInfo->manufacturerId  = 0x115F;  // Aqara/Lumi
    currentImageInfo->imageTypeId     = H2_IMAGE_TYPE_ID;
    currentImageInfo->firmwareVersion = H2_FIRMWARE_VERSION;
  }
  if (hardwareVersion != NULL) {
    *hardwareVersion = 0xFFFF;  // SL_ZIGBEE_AF_INVALID_HARDWARE_VERSION
  }
}
