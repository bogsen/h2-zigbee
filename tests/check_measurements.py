#!/usr/bin/env python3
"""Exercise production measurement/RX/TX code on the host for both variants."""
from pathlib import Path
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
power = (ROOT / 'common/h2_power.c').read_text()
relay = (ROOT / 'common/h2_relay.c').read_text()


def function(source, name):
    match = re.search(rf'^(?:static )?[\w *]+\b{name}\([^;]*?\)\n\{{.*?^\}}',
                      source, re.MULTILINE | re.DOTALL)
    assert match, name
    return match[0]


prefix = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <math.h>
#include <float.h>
#define sl_zigbee_app_debug_println(...) ((void)0)
#define sl_zigbee_app_debug_print(...) ((void)0)
#define H2_SWITCH_GANG_COUNT (H2_HAS_DIMMER ? 1 : 2)
#define SL_ZIGBEE_JOINED_NETWORK 1
#define ZCL_FLOAT_SINGLE_ATTRIBUTE_TYPE 0x39
#define AQARA_MFG_CODE 0x115f
#define OPPLE_CLUSTER_ID 0xfcc0
#define HA_PROFILE_ID 0x0104
#define SL_ZIGBEE_AF_DEFAULT_APS_OPTIONS 0
#define SL_ZIGBEE_APS_OPTION_ENCRYPTION 1
#define SL_ZIGBEE_APS_OPTION_RETRY 2
#define SL_ZIGBEE_OUTGOING_DIRECT 0
#define RELAY_TYPE (H2_HAS_DIMMER ? 0x8007 : 0x8002)
#define RELAY_CMD_SET 2
#define RELAY_CMD_READ 1
#define RELAY_CMD_QUERY 3
#define SL_POWER_MANAGER_EM1 1
#define WALL2_PORT 0
#define WALL2_PIN 0
#define sl_uartdrv_eusart_wall_uart_handle 0
#define ECODE_NVM3_OK 0
#define ECODE_NVM3_ERR_KEY_NOT_FOUND 1
#define nvm3_defaultHandle 0
typedef int Ecode_t;
typedef struct { unsigned delay; } sl_zigbee_af_event_t;
typedef struct {
    uint16_t profileId, clusterId;
    uint8_t sourceEndpoint, destinationEndpoint;
    uint16_t options;
} sl_zigbee_aps_frame_t;
typedef void (*h2_relay_rx_callback_t)(uint16_t, uint8_t, const uint8_t *, uint8_t);
static h2_relay_rx_callback_t rx_callback;
static uint8_t seq, rx_dispatch_buf[128];
static uint16_t rx_dispatch_len;
static bool rx_packet_ready;
uint8_t appResponseData[128];
uint16_t appResponseLength;
static uint32_t clock_ticks;
static bool joined;
static double stored_energy;
static int read_error = ECODE_NVM3_ERR_KEY_NOT_FOUND, write_error;
static unsigned writes, reports, transmissions;
static uint8_t last_tx[128];
static size_t last_tx_len;
static uint32_t sl_sleeptimer_get_tick_count(void) { return clock_ticks; }
static uint32_t sl_sleeptimer_get_timer_frequency(void) { return 32768; }
static int sl_zigbee_af_network_state(void) { return joined; }
static uint8_t sl_zigbee_af_next_sequence(void) { return 42; }
static void sl_zigbee_af_event_init(sl_zigbee_af_event_t *e, void (*fn)(sl_zigbee_af_event_t *)) { e->delay = 0; }
static void sl_zigbee_af_event_set_delay_ms(sl_zigbee_af_event_t *e, unsigned ms) { e->delay = ms; }
static Ecode_t nvm3_readData(int h, unsigned key, void *data, size_t len)
{
    assert(key == 0x0200 && len == sizeof(double));
    if (!read_error) memcpy(data, &stored_energy, len);
    return read_error;
}
static Ecode_t nvm3_writeData(int h, unsigned key, const void *data, size_t len)
{
    assert(key == 0x0200 && len == sizeof(double));
    writes++;
    if (!write_error) { memcpy(&stored_energy, data, len); read_error = 0; }
    return write_error;
}
static void sl_zigbee_af_send_unicast(int kind, uint16_t target,
    const sl_zigbee_aps_frame_t *aps, uint16_t len, const uint8_t *data)
{
    assert(target == 0 && aps->clusterId == 0xfcc0 && aps->sourceEndpoint == 1);
    assert(aps->destinationEndpoint == 1 && aps->profileId == 0x0104);
    assert(len >= 12 && len <= 33 && data[0] == 0x1c);
    assert(data[1] == 0x5f && data[2] == 0x11 && data[4] == 0x0a);
    reports++;
}
static void sl_power_manager_add_em_requirement(int mode) {}
static void sl_power_manager_remove_em_requirement(int mode) {}
static void GPIO_PinOutClear(int port, int pin) {}
static void GPIO_PinOutSet(int port, int pin) {}
static void UARTDRV_TransmitB(int h, const uint8_t *data, size_t len)
{
    memcpy(last_tx, data, len); last_tx_len = len; transmissions++;
}
void h2_relay_send_measurement_query(uint8_t gang);
void h2_power_start(void);
'''
parts = [prefix]
parts += [function(relay, name) for name in [
    'crc16_modbus', 'relay_send_packet', 'h2_relay_set_rx_callback',
    'h2_relay_process_rx', 'h2_relay_send_measurement_query']]
# Keep the entire production translation unit except platform/project includes.
parts.append(re.sub(r'^#include "[^"\n]+"\n', '', power, flags=re.MULTILINE))
parts.append(r'''
static void advance(unsigned seconds) { clock_ticks += seconds * 32768u; }
static void poll(void) { power_poll_handler(&power_poll_event); }
static void feed(uint16_t cmd, float value)
{
    // Feed the production payload parser with a big-endian float response.
    uint8_t bytes[4]; memcpy(bytes, &value, 4);
    memset(rx_dispatch_buf, 0, sizeof(rx_dispatch_buf));
    rx_dispatch_buf[4] = 4; rx_dispatch_buf[6] = 7;
    rx_dispatch_buf[8] = cmd >> 8; rx_dispatch_buf[9] = cmd;
    rx_dispatch_buf[10] = 0x55;
    for (unsigned i = 0; i < 4; i++) rx_dispatch_buf[11 + i] = bytes[3 - i];
    rx_dispatch_len = 17; rx_packet_ready = true;
    h2_relay_process_rx();
    assert(!rx_packet_ready);
}
static float attribute(uint16_t id)
{
    for (unsigned off = 5; off + 7 <= appResponseLength; off += 7) {
        assert(appResponseData[off + 2] == 0x39);
        if ((appResponseData[off] | (appResponseData[off + 1] << 8)) == id) {
            float value; memcpy(&value, &appResponseData[off + 3], 4); return value;
        }
    }
    return NAN;
}
static void reboot(void)
{
    memset(&power, 0, sizeof(power)); memset(&current, 0, sizeof(current));
#if H2_HAS_DIMMER
    memset(&voltage, 0, sizeof(voltage));
#endif
    energy_kwh = 0; energy_dirty = false; energy_available = false;
    report_counter = 0; clock_ticks = 0; writes = reports = 0; joined = false;
    h2_power_init();
    assert(power_poll_event.delay == 1000);
}
int main(void)
{
    reboot();
    assert(energy_available && energy_kwh == 0);
    joined = true;
    report_measurements(clock_ticks);
    assert(isnan(attribute(150)) && isnan(attribute(151)) && isnan(attribute(152)));
    feed(0x0600, 230.0f); feed(0x0601, 100.0f); feed(0x0602, 15.0f);
    report_measurements(clock_ticks);
#if H2_HAS_DIMMER
    assert(attribute(150) == 23000.0f);
#else
    assert(isnan(attribute(150)));
#endif
    assert(attribute(151) == 100.0f && attribute(152) == 15.0f);
    feed(0x0504, 999.0f); feed(0x0608, 999.0f);
    assert(power.value == 15.0f && energy_kwh == 0);
    // READ grammar and actual header/checksums: never a SET or fabricated rw flag.
    h2_relay_send_measurement_query(0);
    const uint8_t query[] = {
#if H2_HAS_DIMMER
        6, 0, 0x55,
#endif
        6, 1, 0x55, 6, 2, 0x55};
    assert(last_tx[0] == 0x55 && last_tx[1] == 0x80);
    assert(last_tx[2] == (H2_HAS_DIMMER ? 7 : 2) && last_tx[4] == 1);
    assert(last_tx[5] == 0 && last_tx[6] == sizeof(query));
    assert(last_tx_len == sizeof(query) + 10 && !memcmp(last_tx + 8, query, sizeof(query)));
    unsigned sum = 0; for (unsigned i = 0; i < 8; i++) sum += last_tx[i];
    assert((sum & 255) == 0);
    uint16_t crc = crc16_modbus(last_tx, last_tx_len - 2);
    assert(last_tx[last_tx_len - 2] == (crc & 255) && last_tx[last_tx_len - 1] == (crc >> 8));
    // Exactly one hour at 15 W, offline, with regular samples and five-minute saves.
    joined = false;
    for (unsigned i = 0; i < 3600; i++) { advance(1); feed(0x0602, 15); poll(); }
    assert(fabs(energy_kwh - 0.015) < 1e-10 && writes == 12);
    assert(fabs(stored_energy - 0.015) < 1e-10);
    reboot();
    assert(fabs(energy_kwh - 0.015) < 1e-10);
    // No energy during a reboot gap, and no extrapolation past freshness timeout.
    double initial = energy_kwh;
    advance(3600); poll(); assert(energy_kwh == initial);
    feed(0x0602, 3600); advance(100); poll();
    assert(fabs(energy_kwh - initial - 0.015) < 1e-10 && !power.valid);
    joined = true; report_measurements(clock_ticks); assert(isnan(attribute(152)));
    // Reconnection and power changes cannot double-count or charge gaps.
    initial = energy_kwh;
    feed(0x0602, 3600); advance(2); feed(0x0602, 7200);
    advance(3); h2_power_start(); poll();
    assert(fabs(energy_kwh - initial - 0.008) < 1e-10);
    // Invalid values invalidate the field and stop integration.
    const float bad[] = {NAN, INFINITY, -1, FLT_MAX};
    for (unsigned i = 0; i < sizeof(bad)/sizeof(bad[0]); i++) {
        feed(0x0602, bad[i]); assert(!power.valid);
        initial = energy_kwh; advance(1); poll(); assert(initial == energy_kwh);
        feed(0x0601, bad[i]); assert(!current.valid);
    }
    // Marker and truncated data must not be accepted as a sample.
    float value = 42; on_measurement_rx(0x0602, 0x80, (uint8_t *)&value, 4);
    on_measurement_rx(0x0602, 0x55, (uint8_t *)&value, 3); assert(!power.valid);
    memset(rx_dispatch_buf, 0, sizeof(rx_dispatch_buf));
    rx_dispatch_buf[6] = 6; rx_dispatch_buf[8] = 6; rx_dispatch_buf[9] = 2;
    rx_dispatch_buf[10] = 0x55; rx_packet_ready = true; h2_relay_process_rx();
    assert(!power.valid);
    // Unknown oversized values must not reach a callback with an 8-byte buffer.
    rx_dispatch_buf[6] = 15; rx_dispatch_buf[8] = 7; rx_dispatch_buf[11] = 12;
    rx_packet_ready = true; h2_relay_process_rx(); assert(!rx_packet_ready);
    // Genuine zero is reported and stops counting; stale V/I are omitted.
    feed(0x0602, 0); feed(0x0601, 0); report_measurements(clock_ticks);
    assert(attribute(151) == 0 && attribute(152) == 0);
    initial = energy_kwh; advance(5); poll(); assert(energy_kwh == initial);
    advance(20); poll(); report_measurements(clock_ticks);
    assert(isnan(attribute(150)) && isnan(attribute(151)) && isnan(attribute(152)));
    // Unsigned elapsed time survives the hardware tick rollover.
    clock_ticks = UINT32_MAX - 32768; energy_tick = save_tick = clock_ticks;
    feed(0x0602, 3600); initial = energy_kwh; advance(3); poll();
    assert(fabs(energy_kwh - initial - 0.003) < 1e-10);
    // Failed saves retain dirty state, retry at the next checkpoint, then survive boot.
    write_error = 2; advance(300); poll(); assert(energy_dirty);
    double old_saved = stored_energy; assert(old_saved < energy_kwh);
    write_error = 0; advance(300); poll(); assert(!energy_dirty && stored_energy == energy_kwh);
    reboot(); assert(energy_kwh == stored_energy);
    // A read error or corrupt total must not overwrite existing storage with zero.
    read_error = 2; reboot(); assert(!energy_available);
    feed(0x0602, 3600); advance(300); poll(); assert(writes == 0);
    joined = true; appResponseLength = 0; report_measurements(clock_ticks);
    assert(isnan(attribute(149)));
    read_error = 0; stored_energy = NAN; reboot(); assert(!energy_available);
    feed(0x0602, 3600); advance(300); poll(); assert(writes == 0);
    return 0;
}
''')
with tempfile.TemporaryDirectory(prefix='h2-measurements-') as tmp:
    source = Path(tmp) / 'measurements.c'
    source.write_text('\n'.join(parts))
    for dimmer in [0, 1]:
        binary = Path(tmp) / f'measurements-{dimmer}'
        subprocess.run(['cc', '-std=c99', '-Wall', '-Werror', '-fsanitize=address,undefined',
                        f'-DH2_HAS_DIMMER={dimmer}', str(source), '-o', str(binary)], check=True)
        subprocess.run([str(binary)], check=True)
print('Switch + dimmer: UART parsing/query, units, freshness, offline energy, persistence, rollover PASS')
