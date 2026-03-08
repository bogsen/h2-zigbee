#include "app/framework/include/af.h"
#include "app/framework/util/common.h"

#include "h2_power.h"
#include "h2_relay.h"
#include "h2_opple.h"
#include "h2_variant.h"

#include <string.h>
#include <math.h>

#define POWER_POLL_MS  1000

// RX command IDs from relay module
#define CMD_STATUS     0x0000
#define CMD_POWER      0x0504
#define CMD_VOLTAGE    0x0601
#define CMD_CURRENT    0x0602
#define CMD_ENERGY     0x0608

// Lumi Opple attribute IDs for measurements (decimal IDs used by Z2M)
// Z2M lumi.ts: 149=energy, 150=voltage, 151=current, 152=power
#define OPPLE_ATTR_ENERGY   0x0095  // 149: energy in kWh (float)
#define OPPLE_ATTR_VOLTAGE  0x0096  // 150: voltage × 10 (uint16, Z2M divides by 10)
#define OPPLE_ATTR_CURRENT  0x0097  // 151: current in mA (uint16, Z2M divides by 1000)
#define OPPLE_ATTR_POWER    0x0098  // 152: power in W (float)

// ZCL frame control constants
#define ZCL_FC_MFG_SPECIFIC        0x04
#define ZCL_FC_DIRECTION_S2C       0x08
#define ZCL_FC_DISABLE_DEFAULT_RSP 0x10
#define ZCL_CMD_REPORT_ATTRIBUTES  0x0A

extern uint8_t h2_active_gang_count;

static volatile uint8_t detected_gang_count;

static float voltage_raw;      // latest voltage from relay module
static float current_raw;      // latest current from relay module
static float energy_kwh;       // energy from announce packet
static bool has_data;          // true after first valid measurement received

// Report only every REPORT_INTERVAL polls to avoid flooding
#define REPORT_INTERVAL  10
static uint8_t report_counter;

static sl_zigbee_af_event_t power_poll_event;

// Declared in framework util/common.c
extern uint8_t appResponseData[];
extern uint16_t appResponseLength;

// ---------------------------------------------------------------------------
// RX callback — receives parsed command entries from h2_relay
// ---------------------------------------------------------------------------

static void on_measurement_rx(uint16_t cmd_id, uint8_t data_type,
                               const uint8_t *data, uint8_t data_len)
{
  // Log ALL commands for diagnostics (helps identify config/calibration data)
  if (data_len >= 4) {
    float val;
    memcpy(&val, data, sizeof(float));
    sl_zigbee_app_debug_println("RX cmd=0x%04X m=0x%02X len=%d "
                                 "raw=%02X%02X%02X%02X val=%d.%02d",
                                 cmd_id, data_type, data_len,
                                 data[0], data[1], data[2], data[3],
                                 (int)val,
                                 (int)(fabsf(val * 100.0f)) % 100);
  } else if (data_len >= 1) {
    sl_zigbee_app_debug_println("RX cmd=0x%04X m=0x%02X len=%d val=0x%02X",
                                 cmd_id, data_type, data_len, data[0]);
  }

  // Status entry (0x0000) in announce: byte[0] (LE) = gang count
  if (cmd_id == CMD_STATUS && data_len >= 1) {
    detected_gang_count = data[0];
    sl_zigbee_app_debug_println("Detected %d gang(s)", detected_gang_count);
  }

  if (data_type != 0x55 || data_len < 4) return;

  float val;
  memcpy(&val, data, sizeof(float));

  switch (cmd_id) {
    case CMD_VOLTAGE:
      voltage_raw = val;
      has_data = true;
      break;
    case CMD_CURRENT:
      current_raw = val;
      break;
    case CMD_ENERGY:
      energy_kwh = val;
      break;
    default:
      break;
  }
}

// ---------------------------------------------------------------------------
// Report measurements via Opple cluster (0xFCC0) attribute reports
// Z2M's lumi fromZigbee converter parses these by attribute ID
// ---------------------------------------------------------------------------

static void append_attr_float(uint16_t attr_id, float val)
{
  appResponseData[appResponseLength++] = (uint8_t)(attr_id & 0xFF);
  appResponseData[appResponseLength++] = (uint8_t)(attr_id >> 8);
  appResponseData[appResponseLength++] = ZCL_FLOAT_SINGLE_ATTRIBUTE_TYPE;  // 0x39
  memcpy(&appResponseData[appResponseLength], &val, 4);
  appResponseLength += 4;
}

static void report_measurements(float power_w)
{
  if (sl_zigbee_af_network_state() != SL_ZIGBEE_JOINED_NETWORK) return;

  appResponseLength = 0;
  // Frame control: global, mfg-specific, server-to-client, disable default rsp
  appResponseData[appResponseLength++] = ZCL_FC_MFG_SPECIFIC
                                        | ZCL_FC_DIRECTION_S2C
                                        | ZCL_FC_DISABLE_DEFAULT_RSP;
  appResponseData[appResponseLength++] = (uint8_t)(AQARA_MFG_CODE & 0xFF);
  appResponseData[appResponseLength++] = (uint8_t)(AQARA_MFG_CODE >> 8);
  appResponseData[appResponseLength++] = sl_zigbee_af_next_sequence();
  appResponseData[appResponseLength++] = ZCL_CMD_REPORT_ATTRIBUTES;

  // energy (0x0095 / 149): kWh as float
  append_attr_float(OPPLE_ATTR_ENERGY, energy_kwh);
#if H2_HAS_DIMMER
  // voltage (0x0096 / 150): KD-R01D Z2M does value * 0.01, so send volts * 100
  append_attr_float(OPPLE_ATTR_VOLTAGE, voltage_raw * 100.0f);
#else
  // voltage (0x0096 / 150): Z2M does value * 0.1, so send volts * 10
  append_attr_float(OPPLE_ATTR_VOLTAGE, voltage_raw * 10.0f);
#endif
  // current (0x0097 / 151): Z2M does value * 0.001; current_raw is centamps
  // centamps * 10 → Z2M divides by 1000 → amps
  append_attr_float(OPPLE_ATTR_CURRENT, current_raw * 10.0f);
  // power (0x0098 / 152): watts as-is
  append_attr_float(OPPLE_ATTR_POWER, power_w);

  sl_zigbee_aps_frame_t aps = {
    .profileId           = HA_PROFILE_ID,
    .clusterId           = OPPLE_CLUSTER_ID,
    .sourceEndpoint      = 1,
    .destinationEndpoint = 1,
    .options             = (SL_ZIGBEE_AF_DEFAULT_APS_OPTIONS
                           | SL_ZIGBEE_APS_OPTION_ENCRYPTION)
                           & ~SL_ZIGBEE_APS_OPTION_RETRY,
  };

  sl_zigbee_af_send_unicast(SL_ZIGBEE_OUTGOING_DIRECT,
                            0x0000,  // coordinator
                            &aps,
                            appResponseLength,
                            appResponseData);

  sl_zigbee_app_debug_println("Report V=%d.%01d I=%d.%03d P=%d.%01d E=%d.%02d",
                               (int)voltage_raw, ((int)(voltage_raw * 10.0f)) % 10,
                               (int)current_raw, ((int)(current_raw * 1000.0f)) % 1000,
                               (int)power_w, ((int)(power_w * 10.0f)) % 10,
                               (int)energy_kwh, ((int)(energy_kwh * 100.0f)) % 100);
}

// ---------------------------------------------------------------------------
// Polling event handler
// ---------------------------------------------------------------------------

static void power_poll_handler(sl_zigbee_af_event_t *event)
{
  (void)event;

  // Send measurement query to relay module
  h2_relay_send_measurement_query(0);

  if (has_data && ++report_counter >= REPORT_INTERVAL) {
    report_counter = 0;

    // current_raw is in centamps (0.01A units)
    float power_w = voltage_raw * current_raw * 0.01f;

    report_measurements(power_w);
  }

  sl_zigbee_af_event_set_delay_ms(&power_poll_event, POWER_POLL_MS);
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void h2_power_init(void)
{
  h2_relay_set_rx_callback(on_measurement_rx);
  sl_zigbee_af_event_init(&power_poll_event, power_poll_handler);
}

uint8_t h2_power_get_detected_gangs(void)
{
  return detected_gang_count;
}


void h2_power_start(void)
{
  // Start polling after a short delay to let init handshake complete
  sl_zigbee_af_event_set_delay_ms(&power_poll_event, POWER_POLL_MS);
}
