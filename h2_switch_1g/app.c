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

#include "h2_relay.h"
#include "h2_button.h"
#include "h2_led.h"
#include "h2_opple.h"

#ifndef H2_SWITCH_GANG_COUNT
#define H2_SWITCH_GANG_COUNT 1
#endif

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

// Events
static sl_zigbee_af_event_t button_tick_event;
static sl_zigbee_af_event_t led_tick_event;
static sl_zigbee_af_event_t finding_and_binding_event;
static sl_zigbee_af_event_t commissioning_event;

//--------------------------------------------------
// Forward declarations
static void button_tick_handler(sl_zigbee_af_event_t *event);
static void led_tick_handler(sl_zigbee_af_event_t *event);
static void finding_and_binding_handler(sl_zigbee_af_event_t *event);
static void commissioning_event_handler(sl_zigbee_af_event_t *event);
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

static void send_to_bindings(void)
{
  sl_zigbee_af_send_command_unicast_to_bindings();
  sl_zigbee_af_send_command_multicast_to_bindings();
}

static void handle_decoupled(uint8_t gang, h2_button_side_t side, h2_button_action_t action)
{
  uint8_t ep = gangs[gang].endpoint;
  sl_zigbee_af_get_command_aps_frame()->sourceEndpoint = ep;

  switch (action) {
    case H2_BTN_ACTION_SINGLE:
      if (side == H2_BTN_SIDE_DOWN) {
        sl_zigbee_af_fill_command_level_control_cluster_move_to_level_with_on_off(254, 10);
      } else {
        sl_zigbee_af_fill_command_on_off_cluster_off();
      }
      send_to_bindings();
      break;

    case H2_BTN_ACTION_LONG:
      if (side == H2_BTN_SIDE_DOWN) {
        // Transition to max brightness (turns on at minimum if off)
        sl_zigbee_af_fill_command_level_control_cluster_move_to_level_with_on_off(254, 40);
      } else {
        // Transition to minimum (no on/off to avoid abrupt off at level 0)
        sl_zigbee_af_fill_command_level_control_cluster_move_to_level(0, 40, 0, 0);
      }
      send_to_bindings();
      break;

    case H2_BTN_ACTION_RELEASE:
      sl_zigbee_af_fill_command_level_control_cluster_stop_with_on_off();
      send_to_bindings();
      break;

    case H2_BTN_ACTION_DOUBLE:
      // Double press: report-only (coordinator automation trigger), no ZCL command
      break;
  }
}

//--------------------------------------------------
// Button callback (called from h2_button state machine via tick)

static void on_button_action(uint8_t gang, h2_button_side_t side, h2_button_action_t action)
{
  sl_zigbee_app_debug_println("Button: gang=%d side=%d action=%d", gang, side, action);

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

  // Dispatch based on operation mode
  if (h2_opple_get_op_mode(gang) == OPPLE_OP_MODE_CONTROL_RELAY) {
    handle_coupled(gang, side, action);
  } else {
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
  h2_button_tick();
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

static void finding_and_binding_handler(sl_zigbee_af_event_t *event)
{
  (void)event;
  sl_status_t status = sl_zigbee_af_find_and_bind_initiator_start(SWITCH_ENDPOINT);
  sl_zigbee_app_debug_println("Find and bind initiator start: 0x%02X", status);
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
  (void)mask;
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
      h2_relay_set(g, on_off_val ? H2_RELAY_ON : H2_RELAY_OFF);
      h2_led_update_relay_state(g, on_off_val != 0);
      h2_opple_save_relay_state(g, on_off_val != 0);
      sl_zigbee_app_debug_println("Relay %d: %s", g, on_off_val ? "ON" : "OFF");
      break;
    }
  }
}

//--------------------------------------------------
// Zigbee framework callbacks

void sl_zigbee_af_main_init_cb(void)
{
  // Initialize hardware modules
  h2_relay_init();
  h2_button_init(on_button_action);
  h2_led_init();
  h2_opple_init();

  // Initialize events
  sl_zigbee_af_event_init(&button_tick_event, button_tick_handler);
  sl_zigbee_af_event_init(&led_tick_event, led_tick_handler);
  sl_zigbee_af_event_init(&finding_and_binding_event, finding_and_binding_handler);
  sl_zigbee_af_event_init(&commissioning_event, commissioning_event_handler);

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
// Pre-message-received: intercept cluster 0xFCC0 before framework processing

bool sl_zigbee_af_pre_message_received_cb(sl_zigbee_af_incoming_message_t *msg)
{
  if (msg->apsFrame->clusterId != OPPLE_CLUSTER_ID) {
    return false;
  }

  return h2_opple_handle_message(msg->apsFrame->destinationEndpoint,
                                 msg->source,
                                 msg->message,
                                 msg->msgLen);
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
