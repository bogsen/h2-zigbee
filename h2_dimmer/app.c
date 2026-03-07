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
#include "h2_encoder.h"
#include "h2_led.h"
#include "h2_opple.h"
#include "h2_power.h"
#include "h2_temperature.h"
#include "sl_sleeptimer.h"
#include "em_gpio.h"
#include "pin_config.h"
#define H2_FIRMWARE_VERSION  0x00000007  // increment for each OTA release
#define H2_IMAGE_TYPE_ID     0x7217      // unique for dimmer variant

#define DIMMER_ENDPOINT           1
#define ENCODER_TICK_MS           1
#define LED_TICK_MS               20
#define FINDING_AND_BINDING_DELAY_MS 3000
#define COMMISSIONING_HOLD_MS     5000
#define FACTORY_RESET_HOLD_MS     10000

// Level move rates (units/sec)
#define MOVE_RATE_SLOW  60
#define MOVE_RATE_FAST  254

// Color temp move rate (mireds/sec)
#define COLOR_TEMP_MOVE_RATE  50

// Rotation idle timeout: stop move after no rotation for this long
#define ROTATION_IDLE_MS  200

// Runtime gang count (always 1 for dimmer)
uint8_t h2_active_gang_count = 1;

static bool commissioning = false;
static bool joined = false;

// Rotation tracking for Z2M reporting
static int16_t rotation_accum_angle;
static uint16_t rotation_accum_time_ms;
static bool rotation_active;
static h2_encoder_mode_t rotation_mode;

// Rotation idle timer for stopping move commands
static uint16_t rotation_idle_timer;
static bool move_active;

// Factory reset state (long hold from encoder)
static bool factory_reset_active = false;
static uint16_t factory_reset_timer_ms = 0;

// MCU-side level transition state
#define LC_TICK_MS  50  // transition step interval (ms)

static bool lc_transition_active = false;
static uint8_t lc_target_level;
static int32_t lc_step_per_tick;     // fixed-point 8.8: level change per tick (signed)
static int32_t lc_level_fp;          // fixed-point 8.8: current level during transition
static uint16_t lc_tick_counter;
static uint16_t lc_total_ticks;
static bool lc_off_after = false;    // turn off relay after transition completes

// Events
static sl_zigbee_af_event_t encoder_tick_event;
static sl_zigbee_af_event_t led_tick_event;
static sl_zigbee_af_event_t finding_and_binding_event;
static sl_zigbee_af_event_t commissioning_event;
static sl_zigbee_af_event_t factory_reset_event;
static sl_zigbee_af_event_t light_sync_event;

// Forward declarations
static void encoder_tick_handler(sl_zigbee_af_event_t *event);
static void led_tick_handler(sl_zigbee_af_event_t *event);
static void finding_and_binding_handler(sl_zigbee_af_event_t *event);
static void commissioning_event_handler(sl_zigbee_af_event_t *event);
static void factory_reset_event_handler(sl_zigbee_af_event_t *event);
static void light_sync_handler(sl_zigbee_af_event_t *event);
static void on_encoder_action(h2_encoder_action_t action, int8_t delta,
                               h2_encoder_mode_t mode);
static void lc_stop_transition(void);

// Declared in framework util/common.c
extern uint8_t appResponseData[];

//--------------------------------------------------
// Model identifier: external attribute storage

static const uint8_t model_id[] = "\x12" "lumi.switch.agl011";

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
    uint8_t total = 1 + model_id[0];
    if (total > maxReadLength) {
      return SL_ZIGBEE_ZCL_STATUS_INSUFFICIENT_SPACE;
    }
    memcpy(buffer, model_id, total);
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
// Binding helpers

static void send_to_bindings(void)
{
  appResponseData[0] |= ZCL_DISABLE_DEFAULT_RESPONSE_MASK;
  sl_zigbee_af_get_command_aps_frame()->options &= ~SL_ZIGBEE_APS_OPTION_RETRY;
  sl_zigbee_af_send_command_multicast_to_bindings();
  sl_zigbee_af_send_command_unicast_to_bindings();
}

static void send_cmd(void)
{
  sl_zigbee_af_get_command_aps_frame()->sourceEndpoint = DIMMER_ENDPOINT;
  send_to_bindings();
}

//--------------------------------------------------
// Local dimmer control (via UART to co-processor)

static void sync_dimmer_to_cluster(void)
{
  uint8_t on_off_val = 0;
  sl_zigbee_af_read_server_attribute(DIMMER_ENDPOINT,
    ZCL_ON_OFF_CLUSTER_ID, ZCL_ON_OFF_ATTRIBUTE_ID,
    &on_off_val, sizeof(on_off_val));
  h2_relay_set(0, on_off_val ? H2_RELAY_ON : H2_RELAY_OFF);
  h2_led_update_relay_state(0, on_off_val != 0);
  h2_opple_save_relay_state(0, on_off_val != 0);
}

//--------------------------------------------------
// Coupled mode handlers (direct local dimmer control)

#define COUPLED_LEVEL_STEP  10

// Guard to prevent post_attribute_change_cb from re-sending UART commands
// when coupled mode code already handled the relay
static bool coupled_updating = false;

static void sync_relay_to_cluster(void)
{
  h2_relay_state_t state = h2_relay_get(0);
  uint8_t on_off_val = (state == H2_RELAY_ON) ? 1 : 0;
  coupled_updating = true;
  sl_zigbee_af_write_server_attribute(
    DIMMER_ENDPOINT, ZCL_ON_OFF_CLUSTER_ID, ZCL_ON_OFF_ATTRIBUTE_ID,
    &on_off_val, ZCL_BOOLEAN_ATTRIBUTE_TYPE);
  coupled_updating = false;
  h2_led_update_relay_state(0, state == H2_RELAY_ON);
  h2_opple_save_relay_state(0, state == H2_RELAY_ON);
}

static void handle_coupled_toggle(void)
{
  h2_relay_state_t state = h2_relay_toggle(0);
  sync_relay_to_cluster();
}

static void handle_coupled_rotation(int8_t delta)
{
  int16_t level = (int16_t)h2_relay_get_level(0);
  level += delta * COUPLED_LEVEL_STEP;
  if (level < 1) level = 1;
  if (level > 254) level = 254;

  // Auto-on if currently off and rotating up
  if (h2_relay_get(0) == H2_RELAY_OFF && delta > 0) {
    h2_relay_set(0, H2_RELAY_ON);
    sync_relay_to_cluster();
  }

  lc_stop_transition();
  h2_relay_set_level(0, (uint8_t)level);

  // Update Zigbee Level Control attribute
  uint8_t lvl = (uint8_t)level;
  sl_zigbee_af_write_server_attribute(
    DIMMER_ENDPOINT, ZCL_LEVEL_CONTROL_CLUSTER_ID,
    ZCL_CURRENT_LEVEL_ATTRIBUTE_ID,
    &lvl, ZCL_INT8U_ATTRIBUTE_TYPE);
}

//--------------------------------------------------
// MCU-side level transition helpers

static void lc_transition_tick(void)
{
  lc_tick_counter++;
  uint8_t level;
  if (lc_tick_counter >= lc_total_ticks) {
    lc_transition_active = false;
    level = lc_target_level;
  } else {
    lc_level_fp += lc_step_per_tick;
    level = (uint8_t)(lc_level_fp >> 8);
    if (level < 1) level = 1;
    if (level > 254) level = 254;
  }
  h2_relay_set_level(0, level);
  coupled_updating = true;
  sl_zigbee_af_write_server_attribute(
    DIMMER_ENDPOINT, ZCL_LEVEL_CONTROL_CLUSTER_ID,
    ZCL_CURRENT_LEVEL_ATTRIBUTE_ID,
    &level, ZCL_INT8U_ATTRIBUTE_TYPE);
  coupled_updating = false;

  // Turn off after fade-to-minimum completes
  if (!lc_transition_active && lc_off_after) {
    lc_off_after = false;
    h2_relay_set(0, H2_RELAY_OFF);
    sync_relay_to_cluster();
  }
}

static void lc_start_transition(uint8_t target, uint16_t transition_ms)
{
  uint8_t current = h2_relay_get_level(0);
  if (transition_ms == 0 || target == current) {
    lc_transition_active = false;
    h2_relay_set_level(0, target);
    coupled_updating = true;
    sl_zigbee_af_write_server_attribute(
      DIMMER_ENDPOINT, ZCL_LEVEL_CONTROL_CLUSTER_ID,
      ZCL_CURRENT_LEVEL_ATTRIBUTE_ID,
      &target, ZCL_INT8U_ATTRIBUTE_TYPE);
    coupled_updating = false;
    return;
  }
  lc_total_ticks = transition_ms / LC_TICK_MS;
  if (lc_total_ticks < 1) lc_total_ticks = 1;
  lc_level_fp = (int32_t)current << 8;
  lc_step_per_tick = (((int32_t)target - (int32_t)current) << 8) / (int32_t)lc_total_ticks;
  lc_target_level = target;
  lc_tick_counter = 0;
  lc_transition_active = true;
}

static void lc_stop_transition(void)
{
  lc_transition_active = false;
  lc_off_after = false;
}

//--------------------------------------------------
// Level Control command handler (from bound switches / Z2M)

static void handle_level_control_cmd(uint8_t cmd_id,
                                      const uint8_t *payload, uint8_t len)
{
  switch (cmd_id) {
    case 0x00: // Move to Level
    case 0x04: // Move to Level with On/Off
    {
      if (len < 3) break;
      uint8_t level = payload[0];
      uint16_t transition = payload[1] | ((uint16_t)payload[2] << 8);
      uint16_t transition_ms = transition * 100;  // tenths of sec → ms

      lc_stop_transition();

      if (level == 0 && cmd_id == 0x04) {
        if (transition_ms > 0 && h2_relay_get(0) == H2_RELAY_ON) {
          // Fade to minimum, then turn off
          lc_off_after = true;
          lc_start_transition(1, transition_ms);
        } else {
          h2_relay_set(0, H2_RELAY_OFF);
          sync_relay_to_cluster();
        }
      } else if (level > 0) {
        if (cmd_id == 0x04 && h2_relay_get(0) == H2_RELAY_OFF) {
          h2_relay_set(0, H2_RELAY_ON);
          sync_relay_to_cluster();
        }
        lc_start_transition(level, transition_ms);
      }
      sl_zigbee_app_debug_println("LC: MoveToLevel %d t=%dms", level, transition_ms);
      break;
    }

    case 0x01: // Move
    case 0x05: // Move with On/Off
    {
      if (len < 2) break;
      uint8_t direction = payload[0];  // 0=up, 1=down
      uint8_t rate = payload[1];       // units/sec

      uint8_t current = h2_relay_get_level(0);
      if (current < 1) current = 1;
      uint8_t target = (direction == 0) ? 254 : 1;
      uint16_t distance = (direction == 0) ? (254 - current) : (current - 1);
      uint16_t transition_ms = (rate > 0) ? ((uint32_t)distance * 1000 / rate) : 0;

      if (cmd_id == 0x05 && direction == 0 && h2_relay_get(0) == H2_RELAY_OFF) {
        h2_relay_set(0, H2_RELAY_ON);
        sync_relay_to_cluster();
      }

      lc_start_transition(target, transition_ms);
      sl_zigbee_app_debug_println("LC: Move dir=%d rate=%d t=%dms",
                                   direction, rate, transition_ms);
      break;
    }

    case 0x03: // Stop
    case 0x07: // Stop with On/Off
    {
      lc_stop_transition();
      uint8_t lvl = h2_relay_get_level(0);
      sl_zigbee_app_debug_println("LC: Stop at %d", lvl);
      break;
    }
  }
}

//--------------------------------------------------
// Encoder action callback (decoupled mode helpers)

static void handle_rotation(int8_t delta, h2_encoder_mode_t mode)
{
  if (!joined) return;

  // Track rotation for Z2M reporting
  rotation_accum_angle += delta * 12;  // ~12 degrees per detent
  rotation_accum_time_ms += 10;

  if (!rotation_active) {
    rotation_active = true;
    rotation_mode = mode;

    // Report start_rotating to Z2M
    h2_opple_report_rotation(DIMMER_ENDPOINT, 1,
      rotation_accum_angle, 0, rotation_accum_time_ms, 0, 0);

    // Start move command to bindings
    if (mode == H2_ENC_MODE_BRIGHTNESS) {
      if (delta > 0) {
        sl_zigbee_af_fill_command_level_control_cluster_move_with_on_off(0, MOVE_RATE_SLOW);
      } else {
        sl_zigbee_af_fill_command_level_control_cluster_move(1, MOVE_RATE_SLOW, 0, 0);
      }
      send_cmd();
    } else {
      // Color temp: move_color_temperature
      uint8_t move_mode = (delta > 0) ? 1 : 3;  // 1=up (warmer), 3=down (cooler)
      sl_zigbee_af_fill_command_color_control_cluster_move_color_temperature(
        move_mode, COLOR_TEMP_MOVE_RATE, 153, 500, 0, 0);
      send_cmd();
    }
  }

  // Report rotation to Z2M
  h2_opple_report_rotation(DIMMER_ENDPOINT, 2,
    rotation_accum_angle, 0, rotation_accum_time_ms, 0, 0);

  // Reset idle timer
  rotation_idle_timer = 0;
  move_active = true;
}

static void stop_rotation(void)
{
  if (!rotation_active) return;

  // Report stop_rotating to Z2M
  h2_opple_report_rotation(DIMMER_ENDPOINT, 3,
    rotation_accum_angle, 0, rotation_accum_time_ms, 0, 0);

  // Send stop command to bindings
  if (rotation_mode == H2_ENC_MODE_BRIGHTNESS) {
    sl_zigbee_af_fill_command_level_control_cluster_stop_with_on_off();
    send_cmd();
  } else {
    sl_zigbee_af_fill_command_color_control_cluster_stop_move_step(0, 0);
    send_cmd();
  }

  rotation_active = false;
  rotation_accum_angle = 0;
  rotation_accum_time_ms = 0;
  move_active = false;
}

static void on_encoder_action(h2_encoder_action_t action, int8_t delta,
                               h2_encoder_mode_t mode)
{
  sl_zigbee_app_debug_println("ENC: action=%d delta=%d mode=%d", action, delta, mode);

  bool coupled = (h2_opple_get_op_mode(0) == OPPLE_OP_MODE_CONTROL_RELAY);

  switch (action) {
    case H2_ENC_ACTION_ROTATE:
      // Any rotation cancels factory reset (must hold still for 15s)
      if (factory_reset_active) {
        factory_reset_active = false;
        h2_opple_init_leds();
      }
      if (coupled) {
        if (mode == H2_ENC_MODE_BRIGHTNESS) {
          handle_coupled_rotation(delta);
        }
        // Color temp rotation ignored in coupled mode
      } else {
        handle_rotation(delta, mode);
      }
      break;

    case H2_ENC_ACTION_TOGGLE:
      if (coupled) {
        handle_coupled_toggle();
        h2_opple_report_action(DIMMER_ENDPOINT, 1);
      } else if (joined) {
        // Toggle on/off to bindings
        sl_zigbee_af_fill_command_on_off_cluster_toggle();
        send_cmd();
        h2_opple_report_action(DIMMER_ENDPOINT, 1);
      } else {
        // Not joined — start commissioning
        sl_zigbee_af_event_set_active(&commissioning_event);
      }
      break;

    case H2_ENC_ACTION_LONG_HOLD:
      if (!joined) {
        sl_zigbee_af_event_set_active(&commissioning_event);
      } else {
        // Start factory reset warning
        factory_reset_active = true;
        factory_reset_timer_ms = 0;
        h2_led_set_mode(0, H2_LED_MODE_MANUAL);
        h2_led_set_manual(0, 0, 100);  // red warning
        h2_opple_report_action(DIMMER_ENDPOINT, 0);  // hold
      }
      break;

    case H2_ENC_ACTION_RELEASE:
      if (factory_reset_active) {
        factory_reset_active = false;
        h2_opple_init_leds();  // restore LED
      }
      h2_opple_report_action(DIMMER_ENDPOINT, 255);  // release
      break;

    case H2_ENC_ACTION_DOUBLE:
      if (!coupled && joined) {
        stop_rotation();
        h2_opple_report_action(DIMMER_ENDPOINT, 2);  // double
      }
      break;
  }
}

//--------------------------------------------------
// Periodic tick events

static void encoder_tick_handler(sl_zigbee_af_event_t *event)
{
  (void)event;
  h2_relay_process_rx();
  h2_encoder_tick();

  // MCU-side level transition tick (every LC_TICK_MS)
  {
    static uint8_t lc_tick_divider = 0;
    if (lc_transition_active) {
      if (++lc_tick_divider >= LC_TICK_MS) {
        lc_tick_divider = 0;
        lc_transition_tick();
      }
    } else {
      lc_tick_divider = 0;
    }
  }

  // Rotation idle detection (check every 10ms via the FSM divider)
  if (move_active) {
    rotation_idle_timer++;
    if (rotation_idle_timer >= (ROTATION_IDLE_MS / 1)) {
      stop_rotation();
    }
  }

  // Factory reset timer (while held)
  if (factory_reset_active) {
    factory_reset_timer_ms++;
    if (factory_reset_timer_ms >= (FACTORY_RESET_HOLD_MS / 1)) {
      sl_zigbee_af_event_set_active(&factory_reset_event);
      factory_reset_active = false;
      factory_reset_timer_ms = 0;
    }
  }

  sl_zigbee_af_event_set_delay_ms(&encoder_tick_event, ENCODER_TICK_MS);
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

  h2_led_set_mode(0, H2_LED_MODE_MANUAL);
  h2_led_set_manual(0, 100, 100);  // blue flash

  sl_zigbee_clear_binding_table();
  nvm3_eraseAll(nvm3_defaultHandle);
  sl_zigbee_leave_network(SL_ZIGBEE_LEAVE_NWK_WITH_NO_OPTION);
  halReboot();
}

static void finding_and_binding_handler(sl_zigbee_af_event_t *event)
{
  (void)event;
  sl_status_t status = sl_zigbee_af_find_and_bind_initiator_start(DIMMER_ENDPOINT);
  sl_zigbee_app_debug_println("Find and bind initiator start: 0x%02X", status);
}

//--------------------------------------------------
// Pre-attribute-change: reject self-loopback

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
  (void)mask; (void)manufacturerCode; (void)type; (void)size; (void)value;

  if ((clusterId == ZCL_ON_OFF_CLUSTER_ID && attributeId == ZCL_ON_OFF_ATTRIBUTE_ID)
      || (clusterId == ZCL_LEVEL_CONTROL_CLUSTER_ID
          && attributeId == ZCL_CURRENT_LEVEL_ATTRIBUTE_ID)) {
    const sl_zigbee_af_cluster_command_t *cmd = sl_zigbee_af_current_command();
    if (cmd != NULL && cmd->source == sl_zigbee_af_get_node_id()) {
      sl_zigbee_app_debug_println("Rejected self-loopback on EP%d cluster 0x%04X",
                                   endpoint, clusterId);
      return SL_ZIGBEE_ZCL_STATUS_FAILURE;
    }
  }
  return SL_ZIGBEE_ZCL_STATUS_SUCCESS;
}

//--------------------------------------------------
// Post-attribute-change: update local dimmer + LED

void sl_zigbee_af_post_attribute_change_cb(uint8_t endpoint,
                                            sl_zigbee_af_cluster_id_t clusterId,
                                            sl_zigbee_af_attribute_id_t attributeId,
                                            uint8_t mask,
                                            uint16_t manufacturerCode,
                                            uint8_t type,
                                            uint8_t size,
                                            uint8_t *value)
{
  (void)manufacturerCode; (void)type; (void)size;

  if (endpoint != DIMMER_ENDPOINT) return;

  if (clusterId == ZCL_ON_OFF_CLUSTER_ID
      && attributeId == ZCL_ON_OFF_ATTRIBUTE_ID) {
    uint8_t on_off_val = *value;
    if (mask == CLUSTER_MASK_SERVER) {
      // Server-side: command from coordinator or coupled mode → control local dimmer
      if (!coupled_updating) {
        h2_relay_set(0, on_off_val ? H2_RELAY_ON : H2_RELAY_OFF);
        h2_led_update_relay_state(0, on_off_val != 0);
        h2_opple_save_relay_state(0, on_off_val != 0);
      }
      sl_zigbee_app_debug_println("Dimmer: %s", on_off_val ? "ON" : "OFF");
    } else {
      // Client-side: report from bound light → update LED only
      h2_led_update_light_state(0, on_off_val != 0);
      sl_zigbee_app_debug_println("Light: %s", on_off_val ? "ON" : "OFF");
    }
  }

  // Level Control — forward attribute writes (from Z2M) to UART
  if (clusterId == ZCL_LEVEL_CONTROL_CLUSTER_ID
      && attributeId == ZCL_CURRENT_LEVEL_ATTRIBUTE_ID
      && mask == CLUSTER_MASK_SERVER
      && !coupled_updating) {
    uint8_t level = *value;
    if (level > 0) {
      if (h2_relay_get(0) == H2_RELAY_OFF) {
        h2_relay_set(0, H2_RELAY_ON);
        h2_led_update_relay_state(0, true);
        h2_opple_save_relay_state(0, true);
      }
      lc_stop_transition();
      h2_relay_set_level(0, level);
    }
    sl_zigbee_app_debug_println("Dimmer level: %d", level);
  }
}

//--------------------------------------------------
// Report Attributes callback (from bound lights)

bool sl_zigbee_af_report_attributes_cb(sl_zigbee_af_cluster_id_t clusterId,
                                        uint8_t *buffer,
                                        uint16_t bufLen)
{
  if (clusterId != ZCL_ON_OFF_CLUSTER_ID || bufLen < 4) return false;

  uint16_t attr_id = buffer[0] | ((uint16_t)buffer[1] << 8);
  if (attr_id != ZCL_ON_OFF_ATTRIBUTE_ID) return false;

  uint8_t on_off_val = buffer[3];
  h2_led_update_light_state(0, on_off_val != 0);
  sl_zigbee_app_debug_println("Light report: %s", on_off_val ? "ON" : "OFF");

  return false;
}

//--------------------------------------------------
// Read Attributes Response callback (boot-time sync)

bool sl_zigbee_af_read_attributes_response_cb(sl_zigbee_af_cluster_id_t clusterId,
                                               uint8_t *buffer,
                                               uint16_t bufLen)
{
  if (clusterId != ZCL_ON_OFF_CLUSTER_ID || bufLen < 5) return false;

  uint16_t attr_id = buffer[0] | ((uint16_t)buffer[1] << 8);
  if (attr_id != ZCL_ON_OFF_ATTRIBUTE_ID || buffer[2] != 0) return false;

  uint8_t on_off_val = buffer[4];
  h2_led_update_light_state(0, on_off_val != 0);
  sl_zigbee_app_debug_println("Light sync: %s", on_off_val ? "ON" : "OFF");

  return false;
}

//--------------------------------------------------
// Light state sync on boot

static void light_sync_handler(sl_zigbee_af_event_t *event)
{
  (void)event;
  if (h2_opple_get_op_mode(0) != OPPLE_OP_MODE_CONTROL_RELAY) {
    uint8_t attr_ids[] = {
      LOW_BYTE(ZCL_ON_OFF_ATTRIBUTE_ID),
      HIGH_BYTE(ZCL_ON_OFF_ATTRIBUTE_ID)
    };
    sl_zigbee_af_fill_command_global_client_to_server_read_attributes(
        ZCL_ON_OFF_CLUSTER_ID, attr_ids, sizeof(attr_ids));
    sl_zigbee_af_get_command_aps_frame()->sourceEndpoint = DIMMER_ENDPOINT;
    sl_zigbee_af_send_command_multicast_to_bindings();
    sl_zigbee_af_send_command_unicast_to_bindings();
  }
}

//--------------------------------------------------
// Pre-message-received: intercept Opple + filter binding On/Off in decoupled mode

bool sl_zigbee_af_pre_message_received_cb(sl_zigbee_af_incoming_message_t *msg)
{
  // Handle Opple cluster (0xFCC0)
  if (msg->apsFrame->clusterId == OPPLE_CLUSTER_ID) {
    return h2_opple_handle_message(msg->apsFrame->destinationEndpoint,
                                   msg->source,
                                   msg->message,
                                   msg->msgLen);
  }

  // In decoupled mode, consume binding-based On/Off cluster-specific commands
  if (msg->apsFrame->clusterId == ZCL_ON_OFF_CLUSTER_ID
      && msg->apsFrame->destinationEndpoint == 0xFF
      && msg->msgLen >= 3
      && (msg->message[0] & 0x03) == 0x01) {
    if (h2_opple_get_op_mode(0) != OPPLE_OP_MODE_CONTROL_RELAY) {
      uint8_t cmd_id = msg->message[2];
      if (cmd_id == 0x01) {
        h2_led_update_light_state(0, true);
      } else if (cmd_id == 0x00) {
        h2_led_update_light_state(0, false);
      }
      return true;  // consume
    }
  }

  // Handle Level Control cluster-specific commands → control local dimmer
  if (msg->apsFrame->clusterId == ZCL_LEVEL_CONTROL_CLUSTER_ID
      && msg->msgLen >= 3
      && (msg->message[0] & 0x03) == 0x01) {
    handle_level_control_cmd(msg->message[2], &msg->message[3], msg->msgLen - 3);
    return true;
  }

  return false;
}

//--------------------------------------------------
// Zigbee framework callbacks

void sl_zigbee_af_main_init_cb(void)
{
  // Erase storage slot 0 so stale OTA images don't get re-applied
  bootloader_eraseStorageSlot(0);

  // Initialize relay UART + power RX callback
  h2_relay_init();
  h2_power_init();

  // Dimmer is always 1 gang — no detection needed
  h2_active_gang_count = 1;

  // Initialize modules
  h2_encoder_init(on_encoder_action);
  h2_led_init();
  h2_opple_init();
  h2_temperature_init();

  // Set encoder sensitivity from stored config
  h2_encoder_set_sensitivity(h2_opple_get_sensitivity(0));

  // Send init handshake to dimmer module
  h2_relay_send_init();

  // Initialize events
  sl_zigbee_af_event_init(&encoder_tick_event, encoder_tick_handler);
  sl_zigbee_af_event_init(&led_tick_event, led_tick_handler);
  sl_zigbee_af_event_init(&finding_and_binding_event, finding_and_binding_handler);
  sl_zigbee_af_event_init(&commissioning_event, commissioning_event_handler);
  sl_zigbee_af_event_init(&factory_reset_event, factory_reset_event_handler);
  sl_zigbee_af_event_init(&light_sync_event, light_sync_handler);

  // Reduce APS ACK timeout
  sl_zigbee_set_aps_ack_timeout_ms(500);

  // Start periodic ticks
  sl_zigbee_af_event_set_delay_ms(&encoder_tick_event, ENCODER_TICK_MS);
  sl_zigbee_af_event_set_delay_ms(&led_tick_event, LED_TICK_MS);
}

void sl_zigbee_af_stack_status_cb(sl_status_t status)
{
  if (status == SL_STATUS_NETWORK_UP) {
    sl_zigbee_app_debug_println("Network UP");
    joined = true;

    // Apply power-on behavior for local dimmer
    h2_opple_apply_power_on_behavior(0);
    sync_dimmer_to_cluster();

    // Initialize dimmer level from Zigbee attribute
    {
      uint8_t level = 254;
      sl_zigbee_af_read_server_attribute(DIMMER_ENDPOINT,
        ZCL_LEVEL_CONTROL_CLUSTER_ID, ZCL_CURRENT_LEVEL_ATTRIBUTE_ID,
        &level, sizeof(level));
      if (level < 1) level = 254;
      h2_relay_set_level(0, level);
    }

    // Start OTA client immediately
    sl_zigbee_af_ota_client_start_cb();

    // Start measurement polling
    h2_power_start();
    h2_temperature_start();

    // Sync LED to bound light state
    sl_zigbee_af_event_set_delay_ms(&light_sync_event, 2000);
  } else if (status == SL_STATUS_NETWORK_DOWN) {
    sl_zigbee_app_debug_println("Network DOWN");
    joined = false;
  }
}

void sl_zigbee_af_network_steering_complete_cb(sl_status_t status,
                                               uint8_t totalBeacons,
                                               uint8_t joinAttempts,
                                               uint8_t finalState)
{
  (void)totalBeacons; (void)joinAttempts; (void)finalState;
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
// Button ISR -> encoder

#if defined(SL_CATALOG_SIMPLE_BUTTON_PRESENT)
void sl_button_on_change(const sl_button_t *handle)
{
  if (SL_SIMPLE_BUTTON_INSTANCE(0) == handle) {
    bool pressed = (sl_button_get_state(handle) == SL_SIMPLE_BUTTON_PRESSED);
    h2_encoder_on_button(pressed);
  }
}
#endif

//--------------------------------------------------
// OTA callbacks

void sl_zigbee_af_ota_client_bootload_cb(const sl_zigbee_af_ota_image_id_t *id)
{
  (void)id;
  sl_util_af_slot_manager_boot_slot(0);
}

void sl_zigbee_af_ota_client_version_info_cb(sl_zigbee_af_ota_image_id_t *currentImageInfo,
                                              uint16_t *hardwareVersion)
{
  if (currentImageInfo != NULL) {
    memset(currentImageInfo, 0, sizeof(sl_zigbee_af_ota_image_id_t));
    currentImageInfo->manufacturerId  = 0x115F;
    currentImageInfo->imageTypeId     = H2_IMAGE_TYPE_ID;
    currentImageInfo->firmwareVersion = H2_FIRMWARE_VERSION;
  }
  if (hardwareVersion != NULL) {
    *hardwareVersion = 0xFFFF;
  }
}
