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

#ifndef H2_SWITCH_GANG_COUNT
#define H2_SWITCH_GANG_COUNT 1
#endif

#define H2_FIRMWARE_VERSION  0x00000056  // increment for each OTA release
#define H2_IMAGE_TYPE_ID     0xEDCF      // unique per product variant

#define SWITCH_ENDPOINT           1
#define BUTTON_TICK_MS            10
#define LED_TICK_MS               20
#define FINDING_AND_BINDING_DELAY_MS 3000
#define COMMISSIONING_HOLD_MS     5000
#define FACTORY_RESET_HOLD_MS     10000

// Per-gang runtime state
typedef struct {
  uint8_t endpoint;
} gang_state_t;

static gang_state_t gangs[H2_SWITCH_GANG_COUNT] = {
  { .endpoint = 1 },
#if H2_SWITCH_GANG_COUNT >= 2
  { .endpoint = 2 },
#endif
};

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
// Coupled mode: button controls local relay

static void handle_coupled(uint8_t gang, h2_button_side_t side, h2_button_action_t action)
{
  if (action == H2_BTN_ACTION_RELEASE) {
    return; // no relay action on release in coupled mode
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
  uint8_t ep = gangs[gang].endpoint;

  switch (action) {
    case H2_BTN_ACTION_PRESS:
      // Immediate move on press — near-zero latency dimming
      if (side == H2_BTN_SIDE_DOWN) {
        // move_with_on_off so light turns on if off
        sl_zigbee_af_fill_command_level_control_cluster_move_with_on_off(0, MOVE_RATE_SLOW);
        send_cmd(ep);
        h2_led_update_light_state(gang, true);
      } else {
        // move (without on_off) so light doesn't turn off at level 0 during dim
        sl_zigbee_af_fill_command_level_control_cluster_move(1, MOVE_RATE_SLOW, 0, 0);
        send_cmd(ep);
      }
      break;

    case H2_BTN_ACTION_SINGLE:
      // Quick tap: jump to target level. Implicitly stops any running move.
      // Single command instead of stop + move = less radio congestion.
      if (side == H2_BTN_SIDE_DOWN) {
        sl_zigbee_af_fill_command_level_control_cluster_move_to_level_with_on_off(254, 10);
        send_cmd(ep);
        h2_led_update_light_state(gang, true);
      } else {
        sl_zigbee_af_fill_command_level_control_cluster_move_to_level_with_on_off(0, 10);
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
      break;

    case H2_BTN_ACTION_DOUBLE:
      // Cancel any running move
      sl_zigbee_af_fill_command_level_control_cluster_stop_with_on_off();
      send_cmd(ep);
      break;
  }
}

//--------------------------------------------------
// Button callback (called from h2_button state machine via tick)

static void on_button_action(uint8_t gang, h2_button_side_t side, h2_button_action_t action)
{
  sl_zigbee_app_debug_println("Button: gang=%d side=%d action=%d", gang, side, action);

  // PRESS: only used for low-latency decoupled dimming.
  // Skip coupled mode, commissioning, and reporting (action_map[] has no entry for it).
  if (action == H2_BTN_ACTION_PRESS) {
    if (sl_zigbee_af_network_state() == SL_ZIGBEE_JOINED_NETWORK
        && h2_opple_get_op_mode(gang) != OPPLE_OP_MODE_CONTROL_RELAY) {
      handle_decoupled(gang, side, action);
    }
    return;
  }

  // Coupled mode works even without network
  if (h2_opple_get_op_mode(gang) == OPPLE_OP_MODE_CONTROL_RELAY) {
    handle_coupled(gang, side, action);
  }

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
  sl_status_t status = sl_zigbee_af_find_and_bind_initiator_start(SWITCH_ENDPOINT);
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
      || attributeId != ZCL_ON_OFF_ATTRIBUTE_ID) {
    return;
  }

  uint8_t on_off_val = *value;

  // Find which gang this endpoint belongs to
  for (uint8_t g = 0; g < H2_SWITCH_GANG_COUNT; g++) {
    if (gangs[g].endpoint == endpoint) {
      if (mask == CLUSTER_MASK_SERVER) {
        // Server-side: command from coordinator or local toggle → control relay
        h2_relay_set(g, on_off_val ? H2_RELAY_ON : H2_RELAY_OFF);
        h2_led_update_relay_state(g, on_off_val != 0);
        h2_opple_save_relay_state(g, on_off_val != 0);
        sl_zigbee_app_debug_println("Relay %d: %s", g, on_off_val ? "ON" : "OFF");
      } else {
        // Client-side: attribute report from bound light → update LED only
        h2_led_update_light_state(g, on_off_val != 0);
        sl_zigbee_app_debug_println("Light %d: %s", g, on_off_val ? "ON" : "OFF");
      }
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
  if (clusterId != ZCL_ON_OFF_CLUSTER_ID || bufLen < 4) {
    return false;
  }

  // Report Attributes payload: attr_id (2B LE) + type (1B) + value (1B)
  uint16_t attr_id = buffer[0] | ((uint16_t)buffer[1] << 8);
  if (attr_id != ZCL_ON_OFF_ATTRIBUTE_ID) {
    return false;
  }

  uint8_t on_off_val = buffer[3];

  const sl_zigbee_af_cluster_command_t *cmd = sl_zigbee_af_current_command();
  if (cmd == NULL) {
    return false;
  }

  uint8_t endpoint = cmd->apsFrame->destinationEndpoint;
  for (uint8_t g = 0; g < H2_SWITCH_GANG_COUNT; g++) {
    if (gangs[g].endpoint == endpoint) {
      h2_led_update_light_state(g, on_off_val != 0);
      sl_zigbee_app_debug_println("Light report EP%d: %s", endpoint,
                                   on_off_val ? "ON" : "OFF");
      break;
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
  if (clusterId != ZCL_ON_OFF_CLUSTER_ID || bufLen < 5) {
    return false;
  }

  // Read Attributes Response: attr_id (2B LE) + status (1B) + type (1B) + value (1B)
  uint16_t attr_id = buffer[0] | ((uint16_t)buffer[1] << 8);
  if (attr_id != ZCL_ON_OFF_ATTRIBUTE_ID || buffer[2] != 0) {
    return false;  // wrong attribute or read failed
  }

  uint8_t on_off_val = buffer[4];

  const sl_zigbee_af_cluster_command_t *cmd = sl_zigbee_af_current_command();
  if (cmd == NULL) {
    return false;
  }

  uint8_t endpoint = cmd->apsFrame->destinationEndpoint;
  for (uint8_t g = 0; g < H2_SWITCH_GANG_COUNT; g++) {
    if (gangs[g].endpoint == endpoint) {
      h2_led_update_light_state(g, on_off_val != 0);
      sl_zigbee_app_debug_println("Light sync EP%d: %s", endpoint,
                                   on_off_val ? "ON" : "OFF");
      break;
    }
  }

  return false;
}

//--------------------------------------------------
// Light state sync: read On/Off from bound lights on boot

static void light_sync_handler(sl_zigbee_af_event_t *event)
{
  (void)event;
  for (uint8_t g = 0; g < H2_SWITCH_GANG_COUNT; g++) {
    if (h2_opple_get_op_mode(g) != OPPLE_OP_MODE_CONTROL_RELAY) {
      uint8_t ep = gangs[g].endpoint;
      uint8_t attr_ids[] = {
        LOW_BYTE(ZCL_ON_OFF_ATTRIBUTE_ID),
        HIGH_BYTE(ZCL_ON_OFF_ATTRIBUTE_ID)
      };
      sl_zigbee_af_fill_command_global_client_to_server_read_attributes(
          ZCL_ON_OFF_CLUSTER_ID, attr_ids, sizeof(attr_ids));
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

  // Initialize hardware modules
  h2_relay_init();
  h2_button_init(on_button_action);
  h2_led_init();
  h2_opple_init();
  h2_power_init();
  h2_temperature_init();

  // Send init handshake to relay module (status + relay state queries)
  h2_relay_send_init();
  // Send gang query to trigger config dump (calibration coefficient)
  h2_relay_send_gang_query(0);

  // Initialize events
  sl_zigbee_af_event_init(&button_tick_event, button_tick_handler);
  sl_zigbee_af_event_init(&led_tick_event, led_tick_handler);
  sl_zigbee_af_event_init(&finding_and_binding_event, finding_and_binding_handler);
  sl_zigbee_af_event_init(&commissioning_event, commissioning_event_handler);
  sl_zigbee_af_event_init(&factory_reset_event, factory_reset_event_handler);
  sl_zigbee_af_event_init(&light_sync_event, light_sync_handler);

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
    for (uint8_t g = 0; g < H2_SWITCH_GANG_COUNT; g++) {
      h2_opple_apply_power_on_behavior(g);
      sync_relay_to_cluster(g);
    }
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

  // In decoupled mode, consume On/Off cluster-specific commands sent to ep 0xFF.
  // These arrive from binding-based sync (coordinator or bound lights) and must
  // not toggle the relay. Direct commands to a specific EP (e.g. EP1) pass through.
  // Reports (global cmd 0x0A) are NOT cluster-specific, so they pass through too.
  if (msg->apsFrame->clusterId == ZCL_ON_OFF_CLUSTER_ID
      && msg->apsFrame->destinationEndpoint == 0xFF
      && msg->msgLen >= 3
      && (msg->message[0] & 0x03) == 0x01) {  // cluster-specific frame type
    for (uint8_t g = 0; g < H2_SWITCH_GANG_COUNT; g++) {
      if (h2_opple_get_op_mode(g) != OPPLE_OP_MODE_CONTROL_RELAY) {
        // Update light state from the command for immediate LED feedback
        uint8_t cmd_id = msg->message[2];
        if (cmd_id == 0x01) {       // On
          h2_led_update_light_state(g, true);
        } else if (cmd_id == 0x00) { // Off
          h2_led_update_light_state(g, false);
        }
        return true;  // consume — don't let framework toggle relay
      }
    }
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
  // btn1 = index 0 (up button, gang 0)
  // For 1-gang with only btn1: index 0 for press, treat as up
  // TODO: Update mapping when additional button instances are added
  if (SL_SIMPLE_BUTTON_INSTANCE(0) == handle) {
    h2_button_on_change(0, pressed);  // gang 0, up
  }
#if SL_SIMPLE_BUTTON_COUNT > 1
  else if (SL_SIMPLE_BUTTON_INSTANCE(1) == handle) {
    h2_button_on_change(1, pressed);  // gang 0, down
  }
#endif
#if SL_SIMPLE_BUTTON_COUNT > 2
  else if (SL_SIMPLE_BUTTON_INSTANCE(2) == handle) {
    h2_button_on_change(2, pressed);  // gang 1, up
  }
#endif
#if SL_SIMPLE_BUTTON_COUNT > 3
  else if (SL_SIMPLE_BUTTON_INSTANCE(3) == handle) {
    h2_button_on_change(3, pressed);  // gang 1, down
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
