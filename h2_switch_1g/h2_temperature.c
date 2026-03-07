#include "app/framework/include/af.h"
#include "app/framework/util/common.h"
#include "em_device.h"
#include "h2_temperature.h"
#include "h2_opple.h"

#define TEMP_POLL_MS  30000
#define EMU_TEMP_ZERO_C_IN_KELVIN  273.15f

// Z2M lumi_specific fromZigbee: key 3 = device_temperature (raw int, °C)
#define OPPLE_ATTR_DEVICE_TEMPERATURE  0x0003

// ZCL frame control constants
#define ZCL_FC_MFG_SPECIFIC        0x04
#define ZCL_FC_DIRECTION_S2C       0x08
#define ZCL_FC_DISABLE_DEFAULT_RSP 0x10
#define ZCL_CMD_REPORT_ATTRIBUTES  0x0A

static sl_zigbee_af_event_t temp_poll_event;

// Declared in framework util/common.c
extern uint8_t appResponseData[];
extern uint16_t appResponseLength;

static float get_chip_temperature(void)
{
  return ((float)((EMU->TEMP & (_EMU_TEMP_TEMP_MASK | _EMU_TEMP_TEMPLSB_MASK))
                  >> _EMU_TEMP_TEMPLSB_SHIFT))
         / 4.0f - EMU_TEMP_ZERO_C_IN_KELVIN;
}

static void report_temperature(int32_t temp_int)
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

  // Attribute 0x0003: device_temperature as INT32S
  appResponseData[appResponseLength++] = (uint8_t)(OPPLE_ATTR_DEVICE_TEMPERATURE & 0xFF);
  appResponseData[appResponseLength++] = (uint8_t)(OPPLE_ATTR_DEVICE_TEMPERATURE >> 8);
  appResponseData[appResponseLength++] = ZCL_INT32S_ATTRIBUTE_TYPE;  // 0x2B
  memcpy(&appResponseData[appResponseLength], &temp_int, 4);
  appResponseLength += 4;

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

static void temp_poll_handler(sl_zigbee_af_event_t *event)
{
  (void)event;

  float temp_c = get_chip_temperature() - 5.0f;
  int32_t temp_int = (int32_t)temp_c;

  report_temperature(temp_int);

  sl_zigbee_af_event_set_delay_ms(&temp_poll_event, TEMP_POLL_MS);
}

void h2_temperature_init(void)
{
  sl_zigbee_af_event_init(&temp_poll_event, temp_poll_handler);
}

void h2_temperature_start(void)
{
  // Fire immediately so Z2M gets a value right away
  sl_zigbee_af_event_set_active(&temp_poll_event);
}
