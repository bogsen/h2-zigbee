# create_ota.cmake — Shared OTA image creation script
# Usage: cmake -P create_ota.cmake -DPROJECT_NAME=... -DIMAGE_TYPE=... etc.
#
# Required -D parameters:
#   PROJECT_NAME    — e.g. h2_switch, h2_dimmer
#   IMAGE_TYPE      — e.g. 0xEDCF
#   OTA_STRING      — e.g. "H2 Switch"
#   MANUFACTURER_ID — e.g. 0x115F
#   DEVICE          — e.g. EFR32MG24A410F1536IM48
#   APP_C           — path to app.c (to extract H2_FIRMWARE_VERSION)
#   S37             — path to built .s37 file
#   OUT_DIR         — output directory for .gbl and .ota

# --- Validate required parameters ---
foreach(var PROJECT_NAME IMAGE_TYPE OTA_STRING MANUFACTURER_ID DEVICE APP_C S37 OUT_DIR)
    if(NOT DEFINED ${var})
        message(FATAL_ERROR "Missing required parameter: ${var}")
    endif()
endforeach()

if(NOT EXISTS "${S37}")
    message(FATAL_ERROR "S37 file not found: ${S37}\nBuild the project first.")
endif()

# --- Find Simplicity Commander ---
set(COMMANDER "")

# 1. POST_BUILD_EXE environment variable
if(DEFINED ENV{POST_BUILD_EXE} AND EXISTS "$ENV{POST_BUILD_EXE}")
    set(COMMANDER "$ENV{POST_BUILD_EXE}")
endif()

# 2. slt where commander
if(NOT COMMANDER)
    find_program(SLT_EXE slt)
    if(SLT_EXE)
        execute_process(
            COMMAND "${SLT_EXE}" where commander
            OUTPUT_VARIABLE SLT_COMMANDER_DIR
            OUTPUT_STRIP_TRAILING_WHITESPACE
            RESULT_VARIABLE SLT_RESULT
        )
        if(SLT_RESULT EQUAL 0 AND EXISTS "${SLT_COMMANDER_DIR}/commander")
            set(COMMANDER "${SLT_COMMANDER_DIR}/commander")
        endif()
    endif()
endif()

# 3. macOS default location
if(NOT COMMANDER)
    set(_macos_commander "$ENV{HOME}/.silabs/slt/installs/archive/Commander.app/Contents/MacOS/commander")
    if(EXISTS "${_macos_commander}")
        set(COMMANDER "${_macos_commander}")
    endif()
endif()

# 4. PATH
if(NOT COMMANDER)
    find_program(COMMANDER commander)
endif()

if(NOT COMMANDER)
    message(FATAL_ERROR "Simplicity Commander not found.\nSet POST_BUILD_EXE or install via slt.")
endif()

# --- Read firmware version from app.c ---
file(STRINGS "${APP_C}" FW_VERSION_LINE REGEX "#define H2_FIRMWARE_VERSION")
if(NOT FW_VERSION_LINE)
    message(FATAL_ERROR "Could not find H2_FIRMWARE_VERSION in ${APP_C}")
endif()
string(REGEX MATCH "0x[0-9a-fA-F]+" FW_VERSION "${FW_VERSION_LINE}")
if(NOT FW_VERSION)
    message(FATAL_ERROR "Could not parse H2_FIRMWARE_VERSION value from: ${FW_VERSION_LINE}")
endif()

# --- Create GBL ---
set(GBL_FILE "${OUT_DIR}/${PROJECT_NAME}.gbl")
set(OTA_FILE "${OUT_DIR}/${PROJECT_NAME}.ota")

message(STATUS "Creating OTA image:")
message(STATUS "  Commander:        ${COMMANDER}")
message(STATUS "  Firmware version: ${FW_VERSION}")
message(STATUS "  Manufacturer ID:  ${MANUFACTURER_ID}")
message(STATUS "  Image type:       ${IMAGE_TYPE}")
message(STATUS "")

message(STATUS "Creating GBL...")
execute_process(
    COMMAND "${COMMANDER}" gbl3 create "${GBL_FILE}"
        --app "${S37}" -d "${DEVICE}" --compress lzma --force
    RESULT_VARIABLE GBL_RESULT
)
if(NOT GBL_RESULT EQUAL 0)
    message(FATAL_ERROR "GBL creation failed (exit code ${GBL_RESULT})")
endif()

# --- Create OTA ---
message(STATUS "Creating OTA file...")
execute_process(
    COMMAND "${COMMANDER}" ota create --type zigbee
        --upgrade-image "${GBL_FILE}"
        --manufacturer-id "${MANUFACTURER_ID}"
        --image-type "${IMAGE_TYPE}"
        --firmware-version "${FW_VERSION}"
        --string "${OTA_STRING}"
        -o "${OTA_FILE}"
    RESULT_VARIABLE OTA_RESULT
)
if(NOT OTA_RESULT EQUAL 0)
    message(FATAL_ERROR "OTA creation failed (exit code ${OTA_RESULT})")
endif()

message(STATUS "Done: ${OTA_FILE} (version ${FW_VERSION})")
