#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
INTERFACE="cmsis-dap"
BL_BIN="$SCRIPT_DIR/firmware/bootloader.bin"
APP_BIN="$SCRIPT_DIR/firmware/application.bin"
CFG="$SCRIPT_DIR/openocd_efr32mg24.cfg"
BACKUP_DIR="$SCRIPT_DIR/backups"

BL_ADDR=0x08000000
APP_ADDR=0x08006000
FLASH_BASE=0x08000000
FLASH_SIZE=0x180000  # 1536KB

NO_BACKUP=0
SKIP_CONFIRM=0

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
        --no-backup)
            NO_BACKUP=1
            shift
            ;;
        --yes|-y)
            SKIP_CONFIRM=1
            shift
            ;;
        --help|-h)
            echo "Usage: $0 [options]"
            echo ""
            echo "Options:"
            echo "  --interface, -i <name>   OpenOCD interface (default: cmsis-dap)"
            echo "                           Examples: cmsis-dap, jlink, stlink"
            echo "  --bootloader <path>      Bootloader .bin file"
            echo "  --application <path>     Application .bin file"
            echo "  --no-backup              Skip flash backup attempt"
            echo "  --yes, -y                Skip confirmation prompt"
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

# Step 1: Attempt flash backup
# Tries to connect, halt, and dump the full flash. This only works if the
# device is debug-unlocked. If it fails (e.g. debug-locked), warn and continue.
BACKED_UP=0
if [ "$NO_BACKUP" != "1" ]; then
    mkdir -p "$BACKUP_DIR"
    TIMESTAMP=$(date +%Y%m%d_%H%M%S)
    BACKUP_FILE="$BACKUP_DIR/flash_backup_${TIMESTAMP}.bin"

    echo "--- Attempting flash backup (1536KB) ---"
    if openocd \
        -f "interface/${INTERFACE}.cfg" \
        -f "$CFG" \
        -c "init" \
        -c "reset halt" \
        -c "dump_image $BACKUP_FILE $FLASH_BASE $FLASH_SIZE" \
        -c "resume" \
        -c "shutdown" 2>&1; then

        BACKUP_SIZE=$(wc -c < "$BACKUP_FILE" | tr -d ' ')
        echo "Backup saved: $BACKUP_FILE ($BACKUP_SIZE bytes)"
        if [ "$BACKUP_SIZE" -ne 1572864 ]; then
            echo "WARNING: Expected 1572864 bytes (1536KB), got $BACKUP_SIZE"
        fi
        BACKED_UP=1
    else
        echo ""
        echo "WARNING: Flash backup failed. The device is likely debug-locked."
        echo "         A DCI device erase will be used, which permanently wipes"
        echo "         the stock firmware with no way to recover it."
        # Clean up empty/partial backup file
        rm -f "$BACKUP_FILE"
    fi
    echo ""
else
    echo "Skipping backup (--no-backup)"
    echo ""
fi

# Step 2: Confirmation
if [ "$SKIP_CONFIRM" != "1" ]; then
    echo "=== Summary ==="
    echo "  Erase:   DCI device erase (clears debug lock + ALL flash)"
    echo "  Program:"
    echo "    Bootloader:  $BL_BIN -> $BL_ADDR"
    echo "    Application: $APP_BIN -> $APP_ADDR"
    echo ""
    read -p "Proceed with erase and flash? [y/N] " CONFIRM
    if [[ ! "$CONFIRM" =~ ^[Yy]$ ]]; then
        echo "Aborted."
        exit 0
    fi
    echo ""
fi

# Step 3: DCI device erase (clears debug lock + all flash)
echo "--- DCI device erase ---"
openocd \
    -f "interface/${INTERFACE}.cfg" \
    -f "$CFG" \
    -c "init" \
    -c "efm32s2_dci_device_erase" \
    -c "shutdown"

echo "Waiting for device to reset..."
sleep 5

# Step 4: Program firmware via MSC registers
echo "--- Flash & verify ---"
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
