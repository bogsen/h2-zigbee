#!/usr/bin/env python3
"""Check descriptors and run the real feedback callbacks with a small host SDK fake."""
import json
from pathlib import Path
import re
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parents[1]


def function(source, name):
    match = re.search(
        rf"^(?:static )?[\w *]+\b{name}\([^;]*?\)\n\{{.*?^\}}",
        source, re.MULTILINE | re.DOTALL,
    )
    assert match, name
    return match[0]


FAKE_SDK = r"""
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#define SL_STATUS_OK 0
#define SL_ZIGBEE_MULTICAST_BINDING 3
#define ZCL_ON_OFF_CLUSTER_ID 6
#define ZCL_LEVEL_CONTROL_CLUSTER_ID 8
#define ZCL_ON_OFF_ATTRIBUTE_ID 0
#define ZCL_CURRENT_LEVEL_ATTRIBUTE_ID 0
#define ZCL_FRAME_CONTROL_FRAME_TYPE_MASK 3
#define ZCL_GLOBAL_COMMAND 0
#define ZCL_MANUFACTURER_SPECIFIC_MASK 4
#define ZCL_FRAME_CONTROL_SERVER_TO_CLIENT 8
#define ZCL_REPORT_ATTRIBUTES_COMMAND_ID 10
#define OPPLE_CLUSTER_ID 0xFCC0
#define OPPLE_OP_MODE_CONTROL_RELAY 1
#define CLUSTER_MASK_SERVER 0x40
#define H2_RELAY_OFF 0
#define H2_RELAY_ON 1
#define LOW_BYTE(v) ((v) & 0xFF)
#define HIGH_BYTE(v) ((v) >> 8)
#define sl_zigbee_app_debug_println(...) ((void)0)
typedef uint16_t sl_zigbee_af_cluster_id_t;
typedef uint16_t sl_zigbee_af_attribute_id_t;
typedef int sl_status_t;
typedef int sl_zigbee_af_event_t;
typedef struct { uint8_t type, local, identifier[8]; uint16_t clusterId; }
    sl_zigbee_binding_table_entry_t;
typedef struct { uint8_t sourceEndpoint, destinationEndpoint; uint16_t groupId, clusterId; }
    sl_zigbee_aps_frame_t;
typedef struct { sl_zigbee_aps_frame_t *apsFrame; } sl_zigbee_af_cluster_command_t;
typedef struct {
    sl_zigbee_aps_frame_t *apsFrame; uint8_t *message; uint16_t msgLen, source;
} sl_zigbee_af_incoming_message_t;
static sl_zigbee_binding_table_entry_t bindings[4];
static sl_zigbee_aps_frame_t aps;
static sl_zigbee_af_cluster_command_t command = { &aps };
static uint8_t levels[2], modes[2], relays[2], sends[8], send_count, find_endpoint, local_level_commands;
static bool coupled_updating;
static uint8_t sl_zigbee_get_binding_table_size(void) { return 4; }
static int sl_zigbee_get_binding(uint8_t i, sl_zigbee_binding_table_entry_t *b)
{ *b = bindings[i]; return 0; }
static bool sl_zigbee_af_groups_cluster_endpoint_in_group_cb(uint8_t ep, uint16_t group)
{
    for (uint8_t i = 0; i < 4; i++)
        if (bindings[i].type == 3 && bindings[i].local == ep
            && (bindings[i].identifier[0] | (bindings[i].identifier[1] << 8)) == group)
            return true;
    return false;
}
static uint16_t sl_zigbee_af_get_node_id(void) { return 0x42; }
static sl_zigbee_af_cluster_command_t *sl_zigbee_af_current_command(void) { return &command; }
static uint8_t h2_opple_get_op_mode(uint8_t gang) { return modes[gang]; }
static void h2_led_update_light_level(uint8_t gang, uint8_t level) { levels[gang] = level; }
static void h2_led_update_light_state(uint8_t gang, bool on) { levels[gang] = on ? 254 : 0; }
static bool h2_opple_handle_message(uint8_t ep, uint16_t src, uint8_t *data, uint16_t len)
{ return false; }
static void handle_level_control_cmd(uint8_t cmd, const uint8_t *data, uint8_t len)
{ local_level_commands++; }
static void h2_relay_set(uint8_t gang, int state) { relays[gang] = state; }
static int h2_relay_get(uint8_t gang) { return relays[gang]; }
static void h2_relay_set_level(uint8_t gang, uint8_t level) {}
static void h2_led_update_relay_state(uint8_t gang, bool state) {}
static void h2_opple_save_relay_state(uint8_t gang, bool state) {}
static uint8_t clamp_to_brightness_range(uint8_t level) { return level; }
static void lc_stop_transition(void) {}
static sl_zigbee_aps_frame_t *sl_zigbee_af_get_command_aps_frame(void) { return &aps; }
static void sl_zigbee_af_fill_command_global_client_to_server_read_attributes(
    uint16_t cluster, uint8_t *attrs, uint8_t len) {}
static void sl_zigbee_af_send_command_multicast_to_bindings(void)
{ sends[send_count++] = aps.sourceEndpoint; }
static void sl_zigbee_af_send_command_unicast_to_bindings(void) {}
static sl_status_t sl_zigbee_af_find_and_bind_initiator_start(uint8_t ep)
{ find_endpoint = ep; return 0; }
"""


CHECKS = r"""
int main(void) {
    uint8_t on_report[] = {0, 0, 0x10, 1};
    uint8_t on_read[] = {0, 0, 0, 0x10, 1};
    uint8_t raw_report[] = {0x18, 0, 10, 0, 0, 0x10, 1};
    uint8_t raw_on[] = {0x11, 0, 1};
    sl_zigbee_af_incoming_message_t msg = {&aps, raw_report, sizeof(raw_report), 0x1234};
    aps.clusterId = 6;
    for (uint8_t g = 0; g < h2_active_gang_count; g++) {
        memset(levels, 0, sizeof(levels));
        aps.destinationEndpoint = 1 + g;
        sl_zigbee_af_report_attributes_cb(6, on_report, sizeof(on_report));
        sl_zigbee_af_read_attributes_response_cb(6, on_read, sizeof(on_read));
        assert(levels[0] == 0 && levels[1] == 0);
        aps.destinationEndpoint = 11 + g;
        sl_zigbee_af_report_attributes_cb(6, on_report, sizeof(on_report));
        assert(levels[g] == 254 && levels[1-g] == 0);
        levels[g] = 0;
        sl_zigbee_af_read_attributes_response_cb(6, on_read, sizeof(on_read));
        assert(levels[g] == 254 && levels[1-g] == 0);

        uint8_t on = 1;
        sl_zigbee_af_post_attribute_change_cb(11 + g, 6, 0, CLUSTER_MASK_SERVER, 0, 0x10, 1, &on);
        assert(relays[g] == 0);
        sl_zigbee_af_post_attribute_change_cb(1 + g, 6, 0, CLUSTER_MASK_SERVER, 0, 0x10, 1, &on);
        assert(relays[g] == 1);
        relays[g] = 0;
    }

    aps.destinationEndpoint = 0xFF;
    aps.groupId = 0x1234;
    // A stale local-output binding must not qualify as a control binding.
    bindings[0] = (sl_zigbee_binding_table_entry_t){3, 1, {0x34, 0x12}, 6};
    memset(levels, 0, sizeof(levels));
    assert(sl_zigbee_af_pre_message_received_cb(&msg));
    assert(levels[0] == 0 && levels[1] == 0);
    bindings[0].local = 11;
    assert(sl_zigbee_af_pre_message_received_cb(&msg));
    assert(levels[0] == 254 && levels[1] == 0);
    memset(levels, 0, sizeof(levels));
    aps.groupId = 0x9999;
    assert(sl_zigbee_af_pre_message_received_cb(&msg));
    assert(levels[0] == 0 && levels[1] == 0);
    aps.groupId = 0x1234;
    msg.source = 0x42;
    assert(sl_zigbee_af_pre_message_received_cb(&msg));
    assert(levels[0] == 0 && levels[1] == 0);
    msg.source = 0x1234;

    msg.message = raw_on; msg.msgLen = sizeof(raw_on);
    if (h2_active_gang_count == 2) {
        bindings[0].local = 12;
        assert(sl_zigbee_af_pre_message_received_cb(&msg));
        assert(levels[0] == 0 && levels[1] == 254);
        bindings[1] = bindings[0]; bindings[1].local = 11;
    }
    memset(levels, 0, sizeof(levels));
    assert(sl_zigbee_af_pre_message_received_cb(&msg));
    assert(levels[0] == 254);
    if (h2_active_gang_count == 2) assert(levels[1] == 254);
    assert(relays[0] == 0 && relays[1] == 0);
    memset(levels, 0, sizeof(levels));
    // Server-to-client and manufacturer-specific commands aren't standard On.
    raw_on[0] = 0x19;
    assert(sl_zigbee_af_pre_message_received_cb(&msg));
    raw_on[0] = 0x15;
    assert(sl_zigbee_af_pre_message_received_cb(&msg));
    raw_on[0] = 0x11;
    aps.groupId = 0x9999;
    assert(sl_zigbee_af_pre_message_received_cb(&msg));
    assert(levels[0] == 0 && levels[1] == 0);
    aps.groupId = 0x1234;
    modes[0] = modes[1] = OPPLE_OP_MODE_CONTROL_RELAY;
    // The switch preserves framework dispatch for coupled relay group control.
    assert(sl_zigbee_af_pre_message_received_cb(&msg) == (bool)DIMMER);
    assert(levels[0] == 0 && levels[1] == 0);
    assert(relays[0] == 0 && relays[1] == 0);

    if (DIMMER) {
        uint8_t level_command[] = {0x11, 0, 0, 100, 0, 0};
        msg.message = level_command; msg.msgLen = sizeof(level_command);
        aps.clusterId = 8;
        // A control-only group cannot operate the coupled local output.
        assert(sl_zigbee_af_pre_message_received_cb(&msg));
        assert(local_level_commands == 0);
        bindings[0].local = 1;
        assert(sl_zigbee_af_pre_message_received_cb(&msg));
        assert(local_level_commands == 1);
        aps.groupId = 0x9999;
        assert(sl_zigbee_af_pre_message_received_cb(&msg));
        assert(local_level_commands == 1);
        aps.groupId = 0x1234;
        level_command[0] = 0x19;
        assert(sl_zigbee_af_pre_message_received_cb(&msg));
        level_command[0] = 0x15;
        assert(sl_zigbee_af_pre_message_received_cb(&msg));
        level_command[0] = 0x11;
        modes[0] = 0;
        assert(sl_zigbee_af_pre_message_received_cb(&msg));
        assert(local_level_commands == 1);
    }

    modes[0] = modes[1] = 0;
    light_sync_handler(NULL);
    assert(send_count == (DIMMER ? 1 : 2 * h2_active_gang_count));
    for (uint8_t i = 0; i < send_count; i++) assert(sends[i] == 11 + (DIMMER ? 0 : i / 2));
    finding_and_binding_handler(NULL);
    assert(find_endpoint == 11);
    return 0;
}
"""


for variant in ("h2_dimmer", "h2_switch"):
    folder = ROOT / variant
    dimmer = variant == "h2_dimmer"
    config = json.loads((folder / "config/zcl/zcl_config.zap").read_text())
    endpoints = {e["endpointId"]: config["endpointTypes"][e["endpointTypeIndex"]] for e in config["endpoints"]}
    assert set(endpoints) == ({1, 4, 11, 242} if dimmer else {1, 2, 4, 5, 11, 12, 242})
    for ep, endpoint in endpoints.items():
        clusters = {(c["code"], c["side"]) for c in endpoint["clusters"] if c["enabled"]}
        if ep in (11, 12):
            assert {(0, "server"), (3, "client"), (4, "client"), (6, "client"), (8, "client")} <= clusters
            assert not {(6, "server"), (8, "server"), (0xFCC0, "server"), (0x19, "client")} & clusters
            if dimmer:
                assert (0x300, "client") in clusters
        elif ep in (1, 2):
            assert {(6, "server"), (0xFCC0, "server")} <= clusters
            assert not {(6, "client"), (8, "client"), (0x300, "client")} & clusters
    assert (0x19, "client") in {(c["code"], c["side"]) for c in endpoints[1]["clusters"]}

    source = (folder / "app.c").read_text()
    prefix = FAKE_SDK + f"\n#define DIMMER {int(dimmer)}\n"
    prefix += re.search(r"^#define CONTROL_ENDPOINT\s+\d+", source, re.MULTILINE)[0] + "\n"
    if dimmer:
        prefix += "#define DIMMER_ENDPOINT 1\nuint8_t h2_active_gang_count = 1;\n"
        prefix += """
#define BRIGHTNESS_SAVE_MS 500
static bool brightness_restored = true;
static sl_zigbee_af_event_t brightness_save_event;
static void sl_zigbee_af_event_set_delay_ms(sl_zigbee_af_event_t *event, int delay) {}
"""
        assert "sourceEndpoint = CONTROL_ENDPOINT" in function(source, "send_cmd")
        assert "h2_opple_report_rotation(DIMMER_ENDPOINT," in source
        assert "h2_opple_report_action(DIMMER_ENDPOINT," in source
    else:
        prefix += "uint8_t h2_active_gang_count = 2;\n"
        prefix += re.search(r"typedef struct \{\n  uint8_t endpoint;.*?^\};", source, re.MULTILINE | re.DOTALL)[0] + "\n"
        assert "uint8_t ep = gangs[gang].control_endpoint;" in function(source, "handle_decoupled")
        assert "uint8_t report_ep = gangs[gang].endpoint" in source
    names = ["binding_group_id", "has_matching_multicast_binding", "report_applies_to_endpoint",
             "parse_report_attribute_u8", "sl_zigbee_af_post_attribute_change_cb",
             "sl_zigbee_af_report_attributes_cb", "sl_zigbee_af_read_attributes_response_cb",
             "sl_zigbee_af_pre_message_received_cb", "light_sync_handler", "finding_and_binding_handler"]
    program = prefix + "\n".join(function(source, name) for name in names) + CHECKS
    with tempfile.TemporaryDirectory(prefix="h2-endpoint-test-") as tmp:
        cfile, binary = Path(tmp) / "check.c", Path(tmp) / "check"
        cfile.write_text(program)
        subprocess.run(["cc", "-std=c99", "-Wall", "-Werror", "-Wno-unused-function",
                        "-Wno-unused-variable", str(cfile), "-o", str(binary)], check=True)
        subprocess.run([str(binary)], check=True)
        if not dimmer:
            cfile.write_text(program.replace("h2_active_gang_count = 2;", "h2_active_gang_count = 1;"))
            subprocess.run(["cc", "-std=c99", str(cfile), "-o", str(binary)], check=True)
            subprocess.run([str(binary)], check=True)
    print(f"{variant}: endpoint descriptors and feedback routing PASS")
