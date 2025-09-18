# MQTT Component Configuration
# This file controls which MQTT components are built

# Read configuration from mqtt_config.h
file(READ "${CMAKE_CURRENT_SOURCE_DIR}/comms/mqtt_config.h" MQTT_CONFIG_CONTENT)

# Check if OTA is enabled
string(FIND "${MQTT_CONFIG_CONTENT}" "#define CONFIG_MQTT_OTA_ENABLED 1" OTA_ENABLED_POS)

# Base MQTT sources (always included)
set(MQTT_SRCS
    comms/mqtt_client.cpp
)

# Conditionally add OTA sources
if(${OTA_ENABLED_POS} GREATER -1)
    message(STATUS "MQTT OTA enabled - including OTA handler")
    list(APPEND MQTT_SRCS comms/mqtt_ota.cpp)
else()
    message(STATUS "MQTT OTA disabled - excluding OTA handler to save space")
endif()

# Add sources to the component
target_sources(${COMPONENT_LIB} PRIVATE ${MQTT_SRCS})