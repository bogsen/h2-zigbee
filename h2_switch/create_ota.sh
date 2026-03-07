#!/bin/bash
set -euo pipefail

# Resolve paths relative to this script's directory
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
S37="${SCRIPT_DIR}/cmake_gcc/build/base/h2_switch.s37"
APP_C="${SCRIPT_DIR}/app.c"

DEVICE=EFR32MG24A410F1536IM48
MANUFACTURER_ID=0x115F
IMAGE_TYPE=0xEDCF

# Find Simplicity Commander: env var > slt > macOS default > PATH
find_commander() {
  if [ -n "${POST_BUILD_EXE:-}" ] && [ -x "$POST_BUILD_EXE" ]; then
    echo "$POST_BUILD_EXE"
  elif command -v slt &>/dev/null; then
    echo "$(slt where commander)/commander"
  elif [ -x "$HOME/.silabs/slt/installs/archive/Commander.app/Contents/MacOS/commander" ]; then
    echo "$HOME/.silabs/slt/installs/archive/Commander.app/Contents/MacOS/commander"
  elif command -v commander &>/dev/null; then
    command -v commander
  else
    echo ""
  fi
}

COMMANDER="$(find_commander)"
if [ -z "$COMMANDER" ]; then
  echo "Error: Simplicity Commander not found."
  echo "Set POST_BUILD_EXE or install via slt."
  exit 1
fi

if [ ! -f "$S37" ]; then
  echo "Error: $S37 not found. Build the project first."
  exit 1
fi

# Read firmware version from app.c
FW_VERSION=$(grep '#define H2_FIRMWARE_VERSION' "$APP_C" | grep -oE '0x[0-9a-fA-F]+')
if [ -z "$FW_VERSION" ]; then
  echo "Error: Could not read H2_FIRMWARE_VERSION from app.c"
  exit 1
fi

OUT_DIR="${SCRIPT_DIR}/cmake_gcc/build/base"

echo "Creating OTA image:"
echo "  Commander:        $COMMANDER"
echo "  Firmware version: $FW_VERSION"
echo "  Manufacturer ID:  $MANUFACTURER_ID"
echo "  Image type:       $IMAGE_TYPE"
echo ""

# Step 1: S37 -> GBL
echo "Creating GBL..."
"$COMMANDER" gbl3 create "${OUT_DIR}/h2_switch.gbl" \
  --app "$S37" -d $DEVICE --compress lzma --force
echo ""

# Step 2: GBL -> OTA
echo "Creating OTA file..."
"$COMMANDER" ota create --type zigbee \
  --upgrade-image "${OUT_DIR}/h2_switch.gbl" \
  --manufacturer-id $MANUFACTURER_ID \
  --image-type $IMAGE_TYPE \
  --firmware-version "$FW_VERSION" \
  --string "H2 Switch" \
  -o "${OUT_DIR}/h2_switch.ota"
echo ""

echo "Done: ${OUT_DIR}/h2_switch.ota (version $FW_VERSION)"
