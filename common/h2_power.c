#include "app/framework/include/af.h"
#include "app/framework/util/common.h"

#include "h2_power.h"
#include "h2_relay.h"
#include "h2_opple.h"
#include "h2_variant.h"
#include "nvm3_default.h"
#include "sl_sleeptimer.h"

#include <string.h>
#include <math.h>

#define POWER_POLL_MS  1000

// RX command IDs from relay module
#define CMD_STATUS     0x0000
#define CMD_VOLTAGE    0x0600
#define CMD_CURRENT    0x0601
#define CMD_POWER      0x0602

// Lumi Opple attribute IDs for measurements (decimal IDs used by Z2M)
// Z2M lumi.ts: 149=energy, 150=voltage, 151=current, 152=power
#define OPPLE_ATTR_ENERGY   0x0095  // 149: energy in kWh (float)
#define OPPLE_ATTR_VOLTAGE  0x0096  // 150: dimmer volts × 100 (float)
#define OPPLE_ATTR_CURRENT  0x0097  // 151: current in mA (float)
#define OPPLE_ATTR_POWER    0x0098  // 152: power in W (float)

// ZCL frame control constants
#define ZCL_FC_MFG_SPECIFIC        0x04
#define ZCL_FC_DIRECTION_S2C       0x08
#define ZCL_FC_DISABLE_DEFAULT_RSP 0x10
#define ZCL_CMD_REPORT_ATTRIBUTES  0x0A

static volatile uint8_t detected_gang_count;

typedef struct {
  float value;
  uint32_t updated;
  bool valid;
} measurement_t;

#if H2_HAS_DIMMER
static measurement_t voltage;
#endif
static measurement_t current;
static measurement_t power;

#define SAMPLE_MAX_AGE_SECONDS  15
#define ENERGY_SAVE_SECONDS    300
#define NVM3_KEY_ENERGY_KWH     0x0200  // separate from Opple's 0x0100..0x011F

static double energy_kwh;
static bool energy_available;
static bool energy_dirty;
static uint32_t energy_tick;
static uint32_t save_tick;

static bool measurement_fresh(const measurement_t *measurement, uint32_t now)
{
  return measurement->valid
         && (uint32_t)(now - measurement->updated)
            < sl_sleeptimer_get_timer_frequency() * SAMPLE_MAX_AGE_SECONDS;
}

static void accumulate_energy(uint32_t now)
{
  uint32_t frequency = sl_sleeptimer_get_timer_frequency();
  uint32_t age = energy_tick - power.updated;
  uint32_t max_age = frequency * SAMPLE_MAX_AGE_SECONDS;
  if (energy_available && power.valid && age < max_age) {
    uint32_t elapsed = now - energy_tick;
    if (elapsed > max_age - age) elapsed = max_age - age;
    if (elapsed != 0 && power.value > 0) {
      energy_kwh += (double)power.value * elapsed / ((double)frequency * 3600000.0);
      energy_dirty = true;
    }
  }
  // Expire the sample so a timer wrap cannot make an old reading fresh again.
  if (!measurement_fresh(&power, now)) power.valid = false;
  energy_tick = now;
}

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
  // Keep unknown fields visible without interpreting arbitrary bytes as floats.
  if (data_len == 4) {
    sl_zigbee_app_debug_println("RX cmd=0x%04X m=0x%02X raw=%02X%02X%02X%02X",
                               cmd_id, data_type, data[0], data[1], data[2], data[3]);
  }

  if (cmd_id == CMD_STATUS && data_type == 0x80 && data_len == 4) {
    if (data[0] >= 1 && data[0] <= H2_SWITCH_GANG_COUNT) {
      detected_gang_count = data[0];
    }
  }
  if (data_type != 0x55 || data_len != sizeof(float)) return;

  measurement_t *measurement;
  float max_value;
  switch (cmd_id) {
#if H2_HAS_DIMMER
    case CMD_VOLTAGE: measurement = &voltage; max_value = 1000.0f; break;
#endif
    case CMD_CURRENT: measurement = &current; max_value = 100000.0f; break;
    case CMD_POWER:   measurement = &power;   max_value = 100000.0f; break;
    default: return;
  }

  uint32_t now = sl_sleeptimer_get_tick_count();
  if (measurement == &power) accumulate_energy(now);
  float value;
  memcpy(&value, data, sizeof(value));
  // Broad plausibility bounds, not load limits or calibration factors.
  measurement->valid = isfinite(value) && value >= 0 && value <= max_value;
  if (measurement->valid) {
    measurement->value = value;
    measurement->updated = now;
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

static void report_measurements(uint32_t now)
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

  if (energy_available) append_attr_float(OPPLE_ATTR_ENERGY, (float)energy_kwh);
#if H2_HAS_DIMMER
  if (measurement_fresh(&voltage, now)) {
    append_attr_float(OPPLE_ATTR_VOLTAGE, voltage.value * 100.0f);
  }
#endif
  if (measurement_fresh(&current, now)) {
    append_attr_float(OPPLE_ATTR_CURRENT, current.value);
  }
  if (measurement_fresh(&power, now)) {
    append_attr_float(OPPLE_ATTR_POWER, power.value);
  }
  if (appResponseLength == 5) return;

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
}

// ---------------------------------------------------------------------------
// Polling event handler
// ---------------------------------------------------------------------------

static void power_poll_handler(sl_zigbee_af_event_t *event)
{
  (void)event;

  uint32_t now = sl_sleeptimer_get_tick_count();
  accumulate_energy(now);
#if H2_HAS_DIMMER
  if (!measurement_fresh(&voltage, now)) voltage.valid = false;
#endif
  if (!measurement_fresh(&current, now)) current.valid = false;

  if ((uint32_t)(now - save_tick)
      >= sl_sleeptimer_get_timer_frequency() * ENERGY_SAVE_SECONDS) {
    save_tick = now;
    if (energy_dirty) {
      // shortcut: lose up to five minutes on power failure; shorten this interval if needed.
      Ecode_t err = nvm3_writeData(nvm3_defaultHandle, NVM3_KEY_ENERGY_KWH,
                                  &energy_kwh, sizeof(energy_kwh));
      if (err == ECODE_NVM3_OK) {
        energy_dirty = false;
      } else {
        sl_zigbee_app_debug_println("Energy save failed: 0x%lX", (unsigned long)err);
      }
    }
  }

  h2_relay_send_measurement_query(0);
  if (++report_counter >= REPORT_INTERVAL) {
    report_counter = 0;
    report_measurements(now);
  }

  sl_zigbee_af_event_set_delay_ms(&power_poll_event, POWER_POLL_MS);
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void h2_power_init(void)
{
  Ecode_t err = nvm3_readData(nvm3_defaultHandle, NVM3_KEY_ENERGY_KWH,
                             &energy_kwh, sizeof(energy_kwh));
  if (err == ECODE_NVM3_ERR_KEY_NOT_FOUND) energy_kwh = 0;
  energy_available = (err == ECODE_NVM3_OK || err == ECODE_NVM3_ERR_KEY_NOT_FOUND)
                     && isfinite(energy_kwh) && energy_kwh >= 0
                     && energy_kwh <= 1.0e12;
  if (!energy_available) {
    sl_zigbee_app_debug_println("Energy storage unreadable: 0x%lX", (unsigned long)err);
  }
  energy_tick = save_tick = sl_sleeptimer_get_tick_count();
  h2_relay_set_rx_callback(on_measurement_rx);
  sl_zigbee_af_event_init(&power_poll_event, power_poll_handler);
  // Meter even before joining Zigbee; reporting still requires a network.
  h2_power_start();
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
