if(NOT ON_EMBEDDED_DEVICE)
    # Build for testing on host
    target_sources(host_test PRIVATE
        gnss_manager.cpp
    )
    target_include_directories(host_test PRIVATE
        .
    )
else()
    # Build for embedded target
    target_sources(application PRIVATE
        gnss_manager.cpp
    )
    target_include_directories(application PRIVATE
        .
    )
endif()