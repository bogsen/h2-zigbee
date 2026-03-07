# Define minimal required version of CMake.
cmake_minimum_required(VERSION "3.25")

# Project definition
project(
	h2_dimmer
	VERSION 1.0
	LANGUAGES C CXX ASM
)

# Include the definition of the slc_h2_dimmer target,
# which contains the content of the SLC project
include(h2_dimmer.cmake)

add_executable(h2_dimmer
    # Add additional sources here
    "../../common/h2_button.c"
    "../../common/h2_encoder.c"
    "../../common/h2_led.c"
    "../../common/h2_opple.c"
    "../../common/h2_power.c"
    "../../common/h2_relay.c"
    "../../common/h2_temperature.c"
)

target_include_directories(h2_dimmer PUBLIC
    # Add additional include paths here
)

# Include dirs on slc so app.c (compiled under slc) can find common/ headers
target_include_directories(slc PUBLIC
    "../../common"
    ".."
)

target_compile_definitions(h2_dimmer PUBLIC
    # Add additional macros here
)

target_compile_options(h2_dimmer PUBLIC
    # Set additional compiler flags here
)

target_link_options(h2_dimmer PUBLIC
    # Set additional linker flags here
)

# Link with the content defined in the SLC project
target_link_libraries(h2_dimmer PRIVATE
    slc
)

# Include managed project content if available
include(h2_dimmer_project.cmake OPTIONAL RESULT_VARIABLE managed_project)
if(managed_project)
    message(STATUS "Using managed project content from ${managed_project}")
endif()

# Force the gcc linker command
set_target_properties(h2_dimmer PROPERTIES LINKER_LANGUAGE C)

# Create .bin, .hex and .s37 artifacts after building the project
add_custom_command(TARGET h2_dimmer
    POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} ${OBJCOPY_SREC_CMD} "$<TARGET_FILE:h2_dimmer>" "$<TARGET_FILE_DIR:h2_dimmer>/$<TARGET_FILE_BASE_NAME:h2_dimmer>.s37"
    COMMAND ${CMAKE_OBJCOPY} ${OBJCOPY_IHEX_CMD} "$<TARGET_FILE:h2_dimmer>" "$<TARGET_FILE_DIR:h2_dimmer>/$<TARGET_FILE_BASE_NAME:h2_dimmer>.hex"
    COMMAND ${CMAKE_OBJCOPY} ${OBJCOPY_BIN_CMD}  "$<TARGET_FILE:h2_dimmer>" "$<TARGET_FILE_DIR:h2_dimmer>/$<TARGET_FILE_BASE_NAME:h2_dimmer>.bin" 
)

# Run post-build pipeline to perform additional post-processing
if(post_build_command)
add_custom_command(TARGET h2_dimmer
    POST_BUILD
    WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/..
    COMMAND ${post_build_command}
)
endif()

# Create OTA image after build
add_custom_command(TARGET h2_dimmer
    POST_BUILD
    COMMAND ${CMAKE_COMMAND}
        -DPROJECT_NAME=h2_dimmer
        -DIMAGE_TYPE=0x7217
        -DOTA_STRING=H2\ Dimmer
        -DMANUFACTURER_ID=0x115F
        -DDEVICE=EFR32MG24A410F1536IM48
        -DAPP_C=${CMAKE_CURRENT_LIST_DIR}/../app.c
        -DS37=$<TARGET_FILE_DIR:h2_dimmer>/h2_dimmer.s37
        -DOUT_DIR=$<TARGET_FILE_DIR:h2_dimmer>
        -P ${CMAKE_CURRENT_LIST_DIR}/../../common/create_ota.cmake
    COMMENT "Creating OTA image for h2_dimmer..."
    VERBATIM
)