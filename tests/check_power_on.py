#!/usr/bin/env python3
"""Run switch/dimmer startup, attribute callbacks, and UART payload code on the host."""
from pathlib import Path
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]


def function(source, name):
    match = re.search(rf"^(?:static )?[\w *]+\b{name}\([^;]*?\)\n\{{.*?^\}}",
                      source, re.MULTILINE | re.DOTALL)
    assert match, name
    return match[0]


app = (ROOT / 'h2_dimmer/app.c').read_text()
opple = (ROOT / 'common/h2_opple.c').read_text()
relay = (ROOT / 'common/h2_relay.c').read_text()
init = function(app, 'sl_zigbee_af_main_init_cb')
assert 'sl_zigbee_af_event_init(&power_on_event, power_on_handler);' in init
assert 'sl_zigbee_af_event_set_active(&power_on_event);' in init
assert init.index('h2_relay_send_init();') < init.index('sl_zigbee_af_event_set_active(&power_on_event);')
stack = function(app, 'sl_zigbee_af_stack_status_cb')
assert 'power_on' not in stack and 'h2_relay_' not in stack

prefix = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#define H2_HAS_RELAY 0
#define H2_HAS_DIMMER 1
#define DIMMER_ENDPOINT 1
#define ZCL_ON_OFF_CLUSTER_ID 6
#define ZCL_LEVEL_CONTROL_CLUSTER_ID 8
#define ZCL_ON_OFF_ATTRIBUTE_ID 0
#define ZCL_CURRENT_LEVEL_ATTRIBUTE_ID 0
#define ZCL_BOOLEAN_ATTRIBUTE_TYPE 0x10
#define ZCL_INT8U_ATTRIBUTE_TYPE 0x20
#define CLUSTER_MASK_SERVER 0x40
#define NVM3_OFF_RELAY_STATE 7
#define NVM3_KEY(g, off) (off)
#define RELAY_TYPE 0x8007
#define RELAY_CMD_SET 2
#define SL_STATUS_NETWORK_UP 1
#define SL_STATUS_NETWORK_DOWN 2
#define sl_zigbee_app_debug_println(...) ((void)0)
typedef int sl_status_t;
typedef uint16_t sl_zigbee_af_cluster_id_t;
typedef uint16_t sl_zigbee_af_attribute_id_t;
typedef struct { bool active; } sl_zigbee_af_event_t;
typedef enum { H2_RELAY_OFF, H2_RELAY_ON } h2_relay_state_t;
static uint8_t h2_active_gang_count = 1;
static struct { uint8_t power_on_behavior; } gangs[1];
static uint8_t stored_state, min_pct, max_pct, attr_on, attr_level;
static bool led_on, coupled_updating, joined, read_fails;
static h2_relay_state_t relay_state[1];
static uint8_t dimmer_level[1];
static uint16_t on_off_transition_ms = 500;
static uint8_t last_packet[18];
static unsigned state_packets, nvm_writes;
static sl_zigbee_af_event_t light_sync_event;
static uint8_t nvm3_read_u8(int key, uint8_t fallback) { return stored_state; }
static void nvm3_write_u8(int key, uint8_t value) { stored_state = value; nvm_writes++; }
static uint8_t h2_opple_get_min_brightness(uint8_t gang) { return min_pct; }
static uint8_t h2_opple_get_max_brightness(uint8_t gang) { return max_pct; }
static void h2_led_update_relay_state(uint8_t gang, bool on) { led_on = on; }
static void lc_stop_transition(void) {}
static void sl_zigbee_af_event_set_inactive(sl_zigbee_af_event_t *event) { event->active = false; }
static void sl_zigbee_af_event_set_delay_ms(sl_zigbee_af_event_t *event, int delay) {}
static void sl_zigbee_af_ota_client_start_cb(void) {}
static void h2_power_start(void) {}
static void h2_temperature_start(void) {}
static void relay_send_packet(uint16_t type, uint8_t command, const uint8_t *data, uint8_t len)
{
    assert(type == 0x8007 && command == 2);
    if (len == 18) {
        memcpy(last_packet, data, len);
        state_packets++;
    } else assert(len == 14);
}
void sl_zigbee_af_post_attribute_change_cb(uint8_t, sl_zigbee_af_cluster_id_t,
    sl_zigbee_af_attribute_id_t, uint8_t, uint16_t, uint8_t, uint8_t, uint8_t *);
static void sl_zigbee_af_read_server_attribute(uint8_t ep, uint16_t cluster,
    uint16_t attr, uint8_t *value, size_t size)
{
    assert(ep == 1 && cluster == 8 && attr == 0);
    if (!read_fails) *value = attr_level;
}
static void sl_zigbee_af_write_server_attribute(uint8_t ep, uint16_t cluster,
    uint16_t attr, uint8_t *value, uint8_t type)
{
    assert(ep == 1 && attr == 0);
    if (cluster == 6) attr_on = *value;
    else { assert(cluster == 8); attr_level = *value; }
    sl_zigbee_af_post_attribute_change_cb(ep, cluster, attr, CLUSTER_MASK_SERVER,
                                         0, type, 1, value);
}
'''
parts = [prefix]
for source, names in [
    (relay, ['brightness_curve', 'h2_relay_set_level', 'h2_relay_set', 'h2_relay_get']),
    (opple, ['h2_opple_save_relay_state', 'h2_opple_apply_power_on_behavior']),
    (app, ['clamp_to_brightness_range', 'sync_relay_to_cluster', 'power_on_handler',
           'sl_zigbee_af_post_attribute_change_cb', 'sl_zigbee_af_stack_status_cb']),
]:
    parts += [function(source, name) for name in names]
parts.append(r'''
int main(void)
{
    const uint8_t ranges[][2] = {{99, 100}, {10, 50}};
    const uint8_t initial_levels[] = {0, 1, 100, 254, 255};
    for (unsigned r = 0; r < 2; r++)
    for (unsigned l = 0; l < 5; l++)
    for (uint8_t behavior = 0; behavior < 5; behavior++)
    for (uint8_t previous = 0; previous < 2; previous++) {
        gangs[0].power_on_behavior = behavior;
        stored_state = previous;
        min_pct = ranges[r][0]; max_pct = ranges[r][1];
        attr_on = 0; attr_level = initial_levels[l];
        relay_state[0] = H2_RELAY_OFF; dimmer_level[0] = 254;
        led_on = false; coupled_updating = false;
        state_packets = nvm_writes = 0;
        sl_zigbee_af_event_t event = {true};
        bool expected = behavior == 0 || (behavior == 1 && previous)
                         || (behavior == 3 && !previous);
        // No network callback is needed for startup restoration.
        power_on_handler(&event);
        assert(!event.active);
        assert(state_packets == 1 && nvm_writes == 1);
        assert(relay_state[0] == expected && stored_state == expected);
        assert(attr_on == expected && led_on == expected);
        uint8_t level = initial_levels[l];
        if (level == 0 || level == 255) level = 254;
        level = clamp_to_brightness_range(level);
        assert(dimmer_level[0] == level && attr_level == level);
        // Verify the actual UART enable bit and level, not just cached state.
        assert(last_packet[0] == 1 && last_packet[1] == 0 && last_packet[2] == 0x55);
        assert(last_packet[3] == expected);
        assert(last_packet[10] == (expected ? brightness_curve(level) : 0));
        // A later network connection/reconnection must not reapply startup policy.
        sl_zigbee_af_stack_status_cb(SL_STATUS_NETWORK_UP);
        uint8_t changed = !expected;
        sl_zigbee_af_post_attribute_change_cb(1, 6, 0, CLUSTER_MASK_SERVER,
                                              0, 0x10, 1, &changed);
        unsigned packets = state_packets, writes = nvm_writes;
        sl_zigbee_af_stack_status_cb(SL_STATUS_NETWORK_DOWN);
        sl_zigbee_af_stack_status_cb(SL_STATUS_NETWORK_UP);
        assert(state_packets == packets && nvm_writes == writes);
        assert(relay_state[0] == changed && stored_state == changed);
    }
    // A failed attribute read still restores at a valid, clamped brightness.
    read_fails = true; min_pct = 99; max_pct = 100;
    gangs[0].power_on_behavior = 1; stored_state = 1;
    sl_zigbee_af_event_t event = {true};
    power_on_handler(&event);
    assert(last_packet[3] == 1 && last_packet[10] == 254 && attr_on == 1);
    unsigned packets = state_packets;
    h2_opple_apply_power_on_behavior(1);
    assert(state_packets == packets);
    return 0;
}
''')
with tempfile.TemporaryDirectory(prefix='h2-power-on-') as tmp:
    source, binary = Path(tmp) / 'check.c', Path(tmp) / 'check'
    source.write_text('\n'.join(parts))
    subprocess.run(['cc', '-std=c99', '-Wall', '-Werror', str(source), '-o', str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
print('Dimmer startup: on/previous/off/inverted, UART payload, brightness limits, offline boot, reconnect PASS')

# Exercise both switch gangs using the real startup, persistence and UART code.
app = (ROOT / 'h2_switch/app.c').read_text()
init = function(app, 'sl_zigbee_af_main_init_cb')
assert 'sl_zigbee_af_event_init(&power_on_event, power_on_handler);' in init
assert 'sl_zigbee_af_event_set_active(&power_on_event);' in init
assert init.index('h2_opple_init();') < init.index('h2_relay_send_init();') < init.index('sl_zigbee_af_event_set_active(&power_on_event);')
stack = function(app, 'sl_zigbee_af_stack_status_cb')
assert 'power_on' not in stack and 'h2_relay_' not in stack
parts = [r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#define H2_HAS_RELAY 1
#define H2_HAS_DIMMER 0
#define RELAY_TYPE 0x8002
#define RELAY_CMD_SET 2
#define NVM3_OFF_RELAY_STATE 7
#define NVM3_KEY(g, off) (g)
#define ZCL_ON_OFF_CLUSTER_ID 6
#define ZCL_ON_OFF_ATTRIBUTE_ID 0
#define ZCL_BOOLEAN_ATTRIBUTE_TYPE 0x10
#define CLUSTER_MASK_SERVER 0x40
#define SL_STATUS_NETWORK_UP 1
#define SL_STATUS_NETWORK_DOWN 2
#define sl_zigbee_app_debug_println(...) ((void)0)
typedef int sl_status_t;
typedef uint16_t sl_zigbee_af_cluster_id_t;
typedef uint16_t sl_zigbee_af_attribute_id_t;
typedef struct { bool active; } sl_zigbee_af_event_t;
typedef enum { H2_RELAY_OFF, H2_RELAY_ON } h2_relay_state_t;
static uint8_t h2_active_gang_count;
static struct { uint8_t endpoint, power_on_behavior; } gangs[2] = {{1, 0}, {2, 0}};
static uint8_t stored_state[2], attr_on[2], sent_state[2];
static bool led_on[2];
static h2_relay_state_t relay_state[2];
static unsigned packets, writes;
static sl_zigbee_af_event_t light_sync_event;
static uint8_t nvm3_read_u8(int key, uint8_t fallback) { return stored_state[key]; }
static void nvm3_write_u8(int key, uint8_t value) { stored_state[key] = value; writes++; }
static void h2_led_update_relay_state(uint8_t gang, bool on) { led_on[gang] = on; }
static void sl_zigbee_af_event_set_inactive(sl_zigbee_af_event_t *event) { event->active = false; }
static void sl_zigbee_af_event_set_delay_ms(sl_zigbee_af_event_t *event, int delay) {}
static void sl_zigbee_af_ota_client_start_cb(void) {}
static void h2_power_start(void) {}
static void h2_temperature_start(void) {}
static void relay_send_packet(uint16_t type, uint8_t cmd, const uint8_t *data, uint8_t len)
{
    assert(type == 0x8002 && cmd == 2 && len == 4);
    assert(data[0] == 1 && data[1] < h2_active_gang_count && data[2] == 0x55);
    sent_state[data[1]] = data[3]; packets++;
}
void sl_zigbee_af_post_attribute_change_cb(uint8_t, sl_zigbee_af_cluster_id_t,
    sl_zigbee_af_attribute_id_t, uint8_t, uint16_t, uint8_t, uint8_t, uint8_t *);
static void sl_zigbee_af_write_server_attribute(uint8_t ep, uint16_t cluster,
    uint16_t attr, uint8_t *value, uint8_t type)
{
    assert(ep >= 1 && ep <= h2_active_gang_count && cluster == 6 && attr == 0);
    attr_on[ep - 1] = *value;
    sl_zigbee_af_post_attribute_change_cb(ep, cluster, attr, CLUSTER_MASK_SERVER, 0, type, 1, value);
}
''']
for source, names in [
    (relay, ['relay_send_set', 'h2_relay_set', 'h2_relay_get']),
    (opple, ['h2_opple_save_relay_state', 'h2_opple_apply_power_on_behavior']),
    (app, ['sync_relay_to_cluster', 'power_on_handler',
           'sl_zigbee_af_post_attribute_change_cb', 'sl_zigbee_af_stack_status_cb']),
]:
    parts += [function(source, name) for name in names]
parts.append(r'''
int main(void)
{
    for (h2_active_gang_count = 1; h2_active_gang_count <= 2; h2_active_gang_count++)
    for (uint8_t b0 = 0; b0 < 5; b0++)
    for (uint8_t b1 = 0; b1 < 5; b1++)
    for (uint8_t previous = 0; previous < 4; previous++) {
        gangs[0].power_on_behavior = b0; gangs[1].power_on_behavior = b1;
        for (uint8_t g = 0; g < 2; g++) {
            stored_state[g] = (previous >> g) & 1;
            relay_state[g] = H2_RELAY_OFF; attr_on[g] = sent_state[g] = 99;
        }
        sl_zigbee_af_event_t event = {true};
        power_on_handler(&event); // No network callback before restoration.
        assert(!event.active);
        for (uint8_t g = 0; g < h2_active_gang_count; g++) {
            uint8_t b = gangs[g].power_on_behavior, p = (previous >> g) & 1;
            bool expected = b == 0 || (b == 1 && p) || (b == 3 && !p);
            assert(relay_state[g] == expected && stored_state[g] == expected);
            assert(attr_on[g] == expected && led_on[g] == expected && sent_state[g] == expected);
        }
        if (h2_active_gang_count == 1) {
            assert(attr_on[1] == 99 && sent_state[1] == 99);
            assert(stored_state[1] == ((previous >> 1) & 1));
        }
        unsigned before_packets = packets, before_writes = writes;
        sl_zigbee_af_stack_status_cb(SL_STATUS_NETWORK_UP);
        assert(packets == before_packets && writes == before_writes);
        for (uint8_t g = 0; g < h2_active_gang_count; g++) {
            uint8_t changed = !relay_state[g];
            sl_zigbee_af_post_attribute_change_cb(g + 1, 6, 0, CLUSTER_MASK_SERVER, 0, 0x10, 1, &changed);
        }
        before_packets = packets; before_writes = writes;
        sl_zigbee_af_stack_status_cb(SL_STATUS_NETWORK_DOWN);
        sl_zigbee_af_stack_status_cb(SL_STATUS_NETWORK_UP);
        assert(packets == before_packets && writes == before_writes);
    }
    return 0;
}
''')
with tempfile.TemporaryDirectory(prefix='h2-switch-power-on-') as tmp:
    source, binary = Path(tmp) / 'check.c', Path(tmp) / 'check'
    source.write_text('\n'.join(parts))
    subprocess.run(['cc', '-std=c99', '-Wall', '-Werror', '-fsanitize=address,undefined',
                    str(source), '-o', str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
print('Switch startup: 1/2 gangs, on/previous/off/inverted, UART payload, offline boot, reconnect PASS')
