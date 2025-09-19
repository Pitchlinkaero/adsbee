if(NOT ON_EMBEDDED_DEVICE)
    # Build for testing on host.
    add_subdirectory(../gnss gnss_build_host_test)
    target_sources(host_test PRIVATE

    )
    target_include_directories(host_test PRIVATE
        
    )
else()
    # Build for embedded target
    add_subdirectory(esp32)
    add_subdirectory(../gnss gnss_build)
    target_sources(application PRIVATE
        eeprom.cc
        # si4362/si4362.cc
        cc1312/cc1312.cc
    )
    target_include_directories(application PRIVATE
        .
        # si4362
        cc1312
    )
endif()