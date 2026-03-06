#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
INTERFACE="cmsis-dap"
BL_BIN="$SCRIPT_DIR/firmware/bootloader.bin"
APP_BIN="$SCRIPT_DIR/firmware/application.bin"
CFG="$SCRIPT_DIR/openocd_efr32mg24.cfg"

BL_ADDR=0x08000000
APP_ADDR=0x08006000

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --interface|-i)
            INTERFACE="$2"
            shift 2
            ;;
        --bootloader)
            BL_BIN="$2"
            shift 2
            ;;
        --application)
            APP_BIN="$2"
            shift 2
            ;;
        --help|-h)
            echo "Usage: $0 [options]"
            echo ""
            echo "Options:"
            echo "  --interface, -i <name>   OpenOCD interface (default: cmsis-dap)"
            echo "                           Examples: cmsis-dap, jlink, stlink"
            echo "  --bootloader <path>      Bootloader .bin file"
            echo "  --application <path>     Application .bin file"
            echo "  --help, -h               Show this help"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            exit 1
            ;;
    esac
done

# Validate files exist
for f in "$BL_BIN" "$APP_BIN"; do
    if [ ! -f "$f" ]; then
        echo "Error: $f not found"
        exit 1
    fi
done

echo "=== EFR32MG24 Flash Tool ==="
echo "Interface:   $INTERFACE"
echo "Bootloader:  $BL_BIN ($(wc -c < "$BL_BIN" | tr -d ' ') bytes -> $BL_ADDR)"
echo "Application: $APP_BIN ($(wc -c < "$APP_BIN" | tr -d ' ') bytes -> $APP_ADDR)"
echo ""

# Step 1: DCI device erase (clears debug lock + all flash)
echo "--- Step 1: DCI device erase ---"
openocd \
    -f "interface/${INTERFACE}.cfg" \
    -f "$CFG" \
    -c "init" \
    -c "efm32s2_dci_device_erase" \
    -c "shutdown"

echo "Waiting for device to reset..."
sleep 2

# Step 2: Program firmware via MSC registers
# DCI erase already wiped flash; efr32_program handles page erase + word writes.
echo "--- Step 2: Flash & verify ---"
openocd \
    -f "interface/${INTERFACE}.cfg" \
    -f "$CFG" \
    -c "init" \
    -c "reset halt" \
    -c "efr32_msc_init" \
    -c "efr32_program $BL_BIN $BL_ADDR" \
    -c "verify_image $BL_BIN $BL_ADDR bin" \
    -c "efr32_program $APP_BIN $APP_ADDR" \
    -c "verify_image $APP_BIN $APP_ADDR bin" \
    -c "reset run" \
    -c "shutdown"

echo ""
echo "=== Programming complete ==="
