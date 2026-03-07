#include "app/framework/include/af.h"
#include "app/framework/util/common.h"
#include "nvm3_default.h"

#include "h2_opple.h"
#include "h2_led.h"
#include "h2_relay.h"

#include "h2_variant.h"

extern uint8_t h2_active_gang_count;

// ---------------------------------------------------------------------------
// NVM3 key scheme: 0x0100 + (gang * 0x10) + offset
// All keys in user domain (0x00000-0x0FFFF)
// ---------------------------------------------------------------------------

#define NVM3_KEY_BASE               0x0100
#define NVM3_OFF_OP_MODE            0
#define NVM3_OFF_SWITCH_TYPE        1
#define NVM3_OFF_LED_INDICATOR      2
#define NVM3_OFF_FLIP_INDICATOR     3
#define NVM3_OFF_POWER_ON_BEHAVIOR  4
#define NVM3_OFF_LOCK_RELAY         5
#define NVM3_OFF_MULTI_CLICK        6
#define NVM3_OFF_RELAY_STATE        7
#define NVM3_OFF_MODE               8

#define NVM3_KEY(gang, off) (NVM3_KEY_BASE + ((gang) * 0x10) + (off))

// ---------------------------------------------------------------------------
// ZCL constants
// ---------------------------------------------------------------------------

#define ZCL_FC_MFG_SPECIFIC        0x04
#define ZCL_FC_DIRECTION_S2C       0x08
#define ZCL_FC_DISABLE_DEFAULT_RSP 0x10

#define ZCL_CMD_READ_ATTRIBUTES      0x00
#define ZCL_CMD_READ_ATTR_RESPONSE   0x01
#define ZCL_CMD_WRITE_ATTRIBUTES     0x02
#define ZCL_CMD_WRITE_ATTR_RESPONSE  0x04
#define ZCL_CMD_REPORT_ATTRIBUTES    0x0A

#define ZCL_STATUS_SUCCESS               0x00
#define ZCL_STATUS_UNSUPPORTED_ATTRIBUTE 0x86
#define ZCL_STATUS_INVALID_VALUE         0x87
#define ZCL_STATUS_READ_ONLY             0x88

// Response frame control: global, mfg-specific, server-to-client, disable default response
#define RESP_FC (ZCL_FC_MFG_SPECIFIC | ZCL_FC_DIRECTION_S2C | ZCL_FC_DISABLE_DEFAULT_RSP)

// ---------------------------------------------------------------------------
// Per-gang attribute storage (RAM, backed by NVM3 for persistent attrs)
// ---------------------------------------------------------------------------

typedef struct {
  uint8_t mode;              // 0=command, 1=event (Z2M sets to 1 during configure)
  uint8_t switch_type;       // 1=toggle, 2=momentary, 3=none
  uint8_t flip_indicator;    // coupled: invert LED; independent: LED on
  uint8_t operation_mode;    // 0=decoupled, 1=control_relay
  uint8_t led_indicator;     // 0=LED independent, 1=LED coupled to relay
  uint8_t lock_relay;        // LED color: 0=blue, 1=red
  uint8_t multi_click;       // 1=disabled, 2=enabled
  uint8_t power_on_behavior; // 0=on, 1=previous, 2=off, 3=inverted
} opple_gang_t;

static opple_gang_t gangs[H2_SWITCH_GANG_COUNT];

// ---------------------------------------------------------------------------
// NVM3 helpers
// ---------------------------------------------------------------------------

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

// ---------------------------------------------------------------------------
// Attribute lookup — returns pointer to RAM value, ZCL type, and read-only flag.
// All real Aqara H2 attributes use UINT8 (0x20) except led_indicator which is BOOLEAN (0x10).
// ---------------------------------------------------------------------------

static uint8_t *get_attr_ptr(uint8_t gang, uint16_t attr_id,
                             uint8_t *zcl_type, bool *read_only)
{
  opple_gang_t *g = &gangs[gang];
  *read_only = false;

  switch (attr_id) {
    case OPPLE_ATTR_MODE:
      *zcl_type = ZCL_INT8U_ATTRIBUTE_TYPE;
      return &g->mode;
    case OPPLE_ATTR_SWITCH_TYPE:
      *zcl_type = ZCL_INT8U_ATTRIBUTE_TYPE;
      return &g->switch_type;
    case OPPLE_ATTR_FLIP_INDICATOR:
      *zcl_type = ZCL_INT8U_ATTRIBUTE_TYPE;
      return &g->flip_indicator;
    case OPPLE_ATTR_OPERATION_MODE:
      *zcl_type = ZCL_INT8U_ATTRIBUTE_TYPE;
      return &g->operation_mode;
    case OPPLE_ATTR_LED_INDICATOR:
      *zcl_type = ZCL_BOOLEAN_ATTRIBUTE_TYPE;
      return &g->led_indicator;
    case OPPLE_ATTR_LOCK_RELAY:
      *zcl_type = ZCL_INT8U_ATTRIBUTE_TYPE;
      return &g->lock_relay;
    case OPPLE_ATTR_MULTI_CLICK:
      *zcl_type = ZCL_INT8U_ATTRIBUTE_TYPE;
      return &g->multi_click;
    case OPPLE_ATTR_POWER_ON_BEHAVIOR:
      *zcl_type = ZCL_INT8U_ATTRIBUTE_TYPE;
      return &g->power_on_behavior;
    default:
      return NULL;
  }
}

// ---------------------------------------------------------------------------
// Endpoint <-> gang mapping
// ---------------------------------------------------------------------------

static int8_t endpoint_to_gang(uint8_t endpoint)
{
  // Endpoint 1 = gang 0 (up/left), endpoint 4 = gang 0 (down)
  // For 2-gang: endpoint 2 = gang 1, endpoint 5 = gang 1 (down)
  if (endpoint == 1 || endpoint == 4) return 0;
  if ((endpoint == 2 || endpoint == 5) && h2_active_gang_count >= 2) return 1;
  return -1;
}

// ---------------------------------------------------------------------------
// Send helper
// ---------------------------------------------------------------------------

static void send_unicast(uint8_t endpoint, uint16_t dest_node)
{
  sl_zigbee_aps_frame_t aps = {
    .profileId           = HA_PROFILE_ID,
    .clusterId           = OPPLE_CLUSTER_ID,
    .sourceEndpoint      = endpoint,
    .destinationEndpoint = 1,
    .options             = SL_ZIGBEE_AF_DEFAULT_APS_OPTIONS
                         | SL_ZIGBEE_APS_OPTION_ENCRYPTION,
  };

  sl_zigbee_af_send_unicast(SL_ZIGBEE_OUTGOING_DIRECT,
                            dest_node,
                            &aps,
                            appResponseLength,
                            appResponseData);
}

// ---------------------------------------------------------------------------
// Write ZCL response header into appResponseData
// ---------------------------------------------------------------------------

static void write_resp_header(uint8_t seq_no, uint8_t cmd_id)
{
  appResponseLength = 0;
  appResponseData[appResponseLength++] = RESP_FC;
  appResponseData[appResponseLength++] = (uint8_t)(AQARA_MFG_CODE & 0xFF);
  appResponseData[appResponseLength++] = (uint8_t)(AQARA_MFG_CODE >> 8);
  appResponseData[appResponseLength++] = seq_no;
  appResponseData[appResponseLength++] = cmd_id;
}

// ---------------------------------------------------------------------------
// Read Attributes handler
// ---------------------------------------------------------------------------

static void handle_read_attributes(uint8_t endpoint, uint16_t source_node,
                                   uint8_t seq_no, uint8_t gang,
                                   const uint8_t *payload, uint16_t payload_len)
{
  write_resp_header(seq_no, ZCL_CMD_READ_ATTR_RESPONSE);

  uint16_t off = 0;
  while (off + 2 <= payload_len) {
    uint16_t attr_id = payload[off] | ((uint16_t)payload[off + 1] << 8);
    off += 2;

    appResponseData[appResponseLength++] = (uint8_t)(attr_id & 0xFF);
    appResponseData[appResponseLength++] = (uint8_t)(attr_id >> 8);

    uint8_t zcl_type;
    bool read_only;
    uint8_t *val = get_attr_ptr(gang, attr_id, &zcl_type, &read_only);

    if (val == NULL) {
      appResponseData[appResponseLength++] = ZCL_STATUS_UNSUPPORTED_ATTRIBUTE;
    } else {
      appResponseData[appResponseLength++] = ZCL_STATUS_SUCCESS;
      appResponseData[appResponseLength++] = zcl_type;
      appResponseData[appResponseLength++] = *val;
    }
  }

  send_unicast(endpoint, source_node);
}

// ---------------------------------------------------------------------------
// LED indicator logic
// lock_relay  (0x0285): color — 0=blue, 1=red
// led_indicator (0x0203): 0=coupled to relay, 1=independent
// flip_indicator (0x00F0): coupled: invert (LED off when relay on),
//                          independent: LED on
// ---------------------------------------------------------------------------

static void update_gang_leds(uint8_t gang)
{
  if (gang >= h2_active_gang_count) return;
  opple_gang_t *g = &gangs[gang];

  sl_zigbee_app_debug_println("LED cfg g%d: ind=%d flip=%d lock=%d op=%d",
    gang, g->led_indicator, g->flip_indicator, g->lock_relay, g->operation_mode);

  // Colors: default pink(10) for ON-state LED, blue(90) for OFF-state LED
  // lock_relay swaps the two colors
  uint8_t on_color  = g->lock_relay ? 90 : 10;
  uint8_t off_color = g->lock_relay ? 10 : 90;

  if (g->led_indicator) {
    // Coupled: UP LED shows on-state color, DOWN LED shows off-state color
    // flip_indicator swaps which LED is UP vs DOWN
    bool decoupled = (g->operation_mode != OPPLE_OP_MODE_CONTROL_RELAY);
    h2_led_set_mode(gang, decoupled ? H2_LED_MODE_FOLLOW_LIGHT
                                    : H2_LED_MODE_FOLLOW_RELAY);
    h2_led_set_on_color(gang, on_color);
    h2_led_set_off_color(gang, off_color);
    h2_led_set_inverted(gang, g->flip_indicator != 0);
    h2_led_set_enabled(gang, true);
  } else {
    // Decoupled: both LEDs on/off together, controlled by flip_indicator
    h2_led_set_mode(gang, H2_LED_MODE_MANUAL);
    h2_led_set_manual(gang, off_color, 100);
    h2_led_set_enabled(gang, g->flip_indicator != 0);
  }
}

// ---------------------------------------------------------------------------
// Write Attributes — apply side effects
// ---------------------------------------------------------------------------

static void apply_attribute(uint8_t gang, uint16_t attr_id, uint8_t value)
{
  opple_gang_t *g = &gangs[gang];

  switch (attr_id) {
    case OPPLE_ATTR_MODE:
      g->mode = value;
      nvm3_write_u8(NVM3_KEY(gang, NVM3_OFF_MODE), value);
      break;

    case OPPLE_ATTR_SWITCH_TYPE:
      g->switch_type = value;
      nvm3_write_u8(NVM3_KEY(gang, NVM3_OFF_SWITCH_TYPE), value);
      break;

    case OPPLE_ATTR_FLIP_INDICATOR:
      g->flip_indicator = value;
      nvm3_write_u8(NVM3_KEY(gang, NVM3_OFF_FLIP_INDICATOR), value);
      update_gang_leds(gang);
      break;

    case OPPLE_ATTR_OPERATION_MODE:
      g->operation_mode = value;
      nvm3_write_u8(NVM3_KEY(gang, NVM3_OFF_OP_MODE), value);
      update_gang_leds(gang);
      break;

    case OPPLE_ATTR_LED_INDICATOR:
      g->led_indicator = value;
      nvm3_write_u8(NVM3_KEY(gang, NVM3_OFF_LED_INDICATOR), value);
      update_gang_leds(gang);
      break;

    case OPPLE_ATTR_LOCK_RELAY:
      g->lock_relay = value;
      nvm3_write_u8(NVM3_KEY(gang, NVM3_OFF_LOCK_RELAY), value);
      update_gang_leds(gang);
      break;

    case OPPLE_ATTR_MULTI_CLICK:
      g->multi_click = value;
      nvm3_write_u8(NVM3_KEY(gang, NVM3_OFF_MULTI_CLICK), value);
      break;

    case OPPLE_ATTR_POWER_ON_BEHAVIOR:
      g->power_on_behavior = value;
      nvm3_write_u8(NVM3_KEY(gang, NVM3_OFF_POWER_ON_BEHAVIOR), value);
      break;

    default:
      break;
  }
}

static bool validate_value(uint16_t attr_id, uint8_t value)
{
  switch (attr_id) {
    case OPPLE_ATTR_MODE:              return value <= 1;
    case OPPLE_ATTR_SWITCH_TYPE:       return value >= 1 && value <= 3;
    case OPPLE_ATTR_FLIP_INDICATOR:    return value <= 1;
    case OPPLE_ATTR_OPERATION_MODE:    return value <= 1;
    case OPPLE_ATTR_LED_INDICATOR:     return value <= 1;
    case OPPLE_ATTR_LOCK_RELAY:        return value <= 1;
    case OPPLE_ATTR_MULTI_CLICK:       return value >= 1 && value <= 2;
    case OPPLE_ATTR_POWER_ON_BEHAVIOR: return value <= 3;
    default:                           return true;
  }
}

// ---------------------------------------------------------------------------
// Write Attributes handler
// ---------------------------------------------------------------------------

static void handle_write_attributes(uint8_t endpoint, uint16_t source_node,
                                    uint8_t seq_no, uint8_t gang,
                                    const uint8_t *payload, uint16_t payload_len)
{
  uint8_t err_buf[48];
  uint16_t err_len = 0;
  bool all_ok = true;

  uint16_t off = 0;
  while (off + 4 <= payload_len) {
    uint16_t attr_id  = payload[off] | ((uint16_t)payload[off + 1] << 8);
    uint8_t data_type = payload[off + 2];
    uint8_t value     = payload[off + 3];
    off += 4;

    uint8_t zcl_type;
    bool read_only;
    uint8_t *attr_ptr = get_attr_ptr(gang, attr_id, &zcl_type, &read_only);

    uint8_t status;
    if (attr_ptr == NULL) {
      status = ZCL_STATUS_UNSUPPORTED_ATTRIBUTE;
    } else if (read_only) {
      status = ZCL_STATUS_READ_ONLY;
    } else if (data_type != zcl_type) {
      status = ZCL_STATUS_INVALID_VALUE;
    } else if (!validate_value(attr_id, value)) {
      status = ZCL_STATUS_INVALID_VALUE;
    } else {
      apply_attribute(gang, attr_id, value);
      status = ZCL_STATUS_SUCCESS;
    }

    if (status != ZCL_STATUS_SUCCESS) {
      all_ok = false;
      err_buf[err_len++] = status;
      err_buf[err_len++] = (uint8_t)(attr_id & 0xFF);
      err_buf[err_len++] = (uint8_t)(attr_id >> 8);
    }
  }

  write_resp_header(seq_no, ZCL_CMD_WRITE_ATTR_RESPONSE);

  if (all_ok) {
    appResponseData[appResponseLength++] = ZCL_STATUS_SUCCESS;
  } else {
    memcpy(&appResponseData[appResponseLength], err_buf, err_len);
    appResponseLength += err_len;
  }

  send_unicast(endpoint, source_node);
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void h2_opple_init(void)
{
  for (uint8_t g = 0; g < h2_active_gang_count; g++) {
    gangs[g].mode              = nvm3_read_u8(NVM3_KEY(g, NVM3_OFF_MODE),              1); // default: event mode
    gangs[g].switch_type       = nvm3_read_u8(NVM3_KEY(g, NVM3_OFF_SWITCH_TYPE),       2); // momentary
    gangs[g].flip_indicator    = nvm3_read_u8(NVM3_KEY(g, NVM3_OFF_FLIP_INDICATOR),    0);
    gangs[g].operation_mode    = nvm3_read_u8(NVM3_KEY(g, NVM3_OFF_OP_MODE),           1); // control_relay
    gangs[g].led_indicator     = nvm3_read_u8(NVM3_KEY(g, NVM3_OFF_LED_INDICATOR),     1); // ON
    gangs[g].lock_relay        = nvm3_read_u8(NVM3_KEY(g, NVM3_OFF_LOCK_RELAY),        0);
    gangs[g].multi_click       = nvm3_read_u8(NVM3_KEY(g, NVM3_OFF_MULTI_CLICK),       1); // disabled
    gangs[g].power_on_behavior = nvm3_read_u8(NVM3_KEY(g, NVM3_OFF_POWER_ON_BEHAVIOR), 1); // previous
  }

  h2_opple_init_leds();
}

void h2_opple_init_leds(void)
{
  for (uint8_t g = 0; g < h2_active_gang_count; g++) {
    update_gang_leds(g);
  }
}

bool h2_opple_handle_message(uint8_t endpoint,
                             uint16_t source_node,
                             uint8_t *msg,
                             uint16_t msg_len)
{
  if (msg_len < 5) return false;

  uint8_t fc = msg[0];

  // Must be global command (frame type 0x00), manufacturer-specific
  if ((fc & 0x03) != 0x00) return false;
  if (!(fc & ZCL_FC_MFG_SPECIFIC)) return false;

  uint16_t mfg_code = msg[1] | ((uint16_t)msg[2] << 8);
  if (mfg_code != AQARA_MFG_CODE) return false;

  uint8_t seq_no = msg[3];
  uint8_t cmd_id = msg[4];
  const uint8_t *payload = &msg[5];
  uint16_t payload_len = msg_len - 5;

  int8_t gang = endpoint_to_gang(endpoint);
  if (gang < 0) return false;

  switch (cmd_id) {
    case ZCL_CMD_READ_ATTRIBUTES:
      handle_read_attributes(endpoint, source_node, seq_no, gang,
                             payload, payload_len);
      return true;

    case ZCL_CMD_WRITE_ATTRIBUTES:
      handle_write_attributes(endpoint, source_node, seq_no, gang,
                              payload, payload_len);
      return true;

    default:
      return false;
  }
}

void h2_opple_report_action(uint8_t endpoint, uint8_t action_value)
{
  if (sl_zigbee_af_network_state() != SL_ZIGBEE_JOINED_NETWORK) return;

  // Build Report Attributes for genMultistateInput (0x0012) / presentValue (0x0055)
  // Frame: FC(1) + SeqNo(1) + CmdId(1) + [AttrId(2) + Type(1) + Value(2)]
  // presentValue is UINT16 in genMultistateInput
  appResponseLength = 0;
  appResponseData[appResponseLength++] = ZCL_FC_DIRECTION_S2C | ZCL_FC_DISABLE_DEFAULT_RSP;
  appResponseData[appResponseLength++] = sl_zigbee_af_next_sequence();
  appResponseData[appResponseLength++] = ZCL_CMD_REPORT_ATTRIBUTES;

  // presentValue attribute (0x0055), type UINT16 (0x21)
  appResponseData[appResponseLength++] = 0x55;
  appResponseData[appResponseLength++] = 0x00;
  appResponseData[appResponseLength++] = ZCL_INT16U_ATTRIBUTE_TYPE;
  appResponseData[appResponseLength++] = action_value;
  appResponseData[appResponseLength++] = 0x00;

  sl_zigbee_aps_frame_t aps = {
    .profileId           = HA_PROFILE_ID,
    .clusterId           = 0x0012, // genMultistateInput
    .sourceEndpoint      = endpoint,
    .destinationEndpoint = 1,
    .options             = (SL_ZIGBEE_AF_DEFAULT_APS_OPTIONS
                         | SL_ZIGBEE_APS_OPTION_ENCRYPTION)
                         & ~SL_ZIGBEE_APS_OPTION_RETRY,
  };

  sl_zigbee_af_send_unicast(SL_ZIGBEE_OUTGOING_DIRECT,
                            0x0000,
                            &aps,
                            appResponseLength,
                            appResponseData);
}

uint8_t h2_opple_get_op_mode(uint8_t gang)
{
  if (gang >= h2_active_gang_count) return OPPLE_OP_MODE_CONTROL_RELAY;
  return gangs[gang].operation_mode;
}

void h2_opple_save_relay_state(uint8_t gang, bool relay_on)
{
  if (gang >= h2_active_gang_count) return;
  nvm3_write_u8(NVM3_KEY(gang, NVM3_OFF_RELAY_STATE), relay_on ? 1 : 0);
}

void h2_opple_apply_power_on_behavior(uint8_t gang)
{
  if (gang >= h2_active_gang_count) return;

  uint8_t behavior   = gangs[gang].power_on_behavior;
  uint8_t prev_state = nvm3_read_u8(NVM3_KEY(gang, NVM3_OFF_RELAY_STATE), 0);

  h2_relay_state_t target;
  switch (behavior) {
    case 0:  target = H2_RELAY_ON;  break;                                      // on
    case 1:  target = prev_state ? H2_RELAY_ON : H2_RELAY_OFF; break;           // previous
    case 2:  target = H2_RELAY_OFF; break;                                      // off
    case 3:  target = prev_state ? H2_RELAY_OFF : H2_RELAY_ON; break;           // inverted
    default: target = H2_RELAY_OFF; break;
  }

  h2_relay_set(gang, target);
}
