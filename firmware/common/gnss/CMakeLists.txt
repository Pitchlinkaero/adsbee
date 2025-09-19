# GNSS/GPS Module CMakeLists.txt

# Add GNSS library
add_library(gnss_common STATIC
    nmea_parser.cpp
    ubx_parser.cpp
)

# Include directories
target_include_directories(gnss_common PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/..
    ${CMAKE_CURRENT_SOURCE_DIR}/../settings
)

# Platform-specific settings
if(ON_PICO)
    target_link_libraries(gnss_common PUBLIC
        pico_stdlib
        hardware_uart
    )
else()
    # Host build for testing
    target_compile_definitions(gnss_common PUBLIC
        HOST_BUILD
    )
endif()

# Add compile options
target_compile_options(gnss_common PRIVATE
    -Wall
    -Wextra
    -Wno-unused-parameter
)

# Export for other modules
set(GNSS_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR} PARENT_SCOPE)