# cmake/FindPahoMqttCpp.cmake
# This module finds Paho MQTT C++ library and creates the target
# PahoMqttCpp::PahoMqttCpp

if(TARGET PahoMqttCpp::PahoMqttCpp)
    return()
endif()

# Find C++ headers
find_path(PAHO_MQTT_CPP_INCLUDE_DIR
    NAMES mqtt/async_client.h
    PATHS
        /usr/local/include
        /usr/include
        ${CMAKE_SOURCE_DIR}/third_party/paho/include
        $ENV{PAHO_MQTT_CPP_ROOT}/include
)

# Find C headers
find_path(PAHO_MQTT_C_INCLUDE_DIR
    NAMES MQTTAsync.h
    PATHS
        /usr/local/include
        /usr/include
        ${CMAKE_SOURCE_DIR}/third_party/paho/include
        $ENV{PAHO_MQTT_C_ROOT}/include
)

# Find C++ library
find_library(PAHO_MQTT_CPP_LIBRARY
    NAMES paho-mqttpp3
    PATHS
        /usr/local/lib
        /usr/lib
        /usr/lib/aarch64-linux-gnu
        /usr/lib/arm-linux-gnueabihf
        ${CMAKE_SOURCE_DIR}/third_party/paho/lib
        $ENV{PAHO_MQTT_CPP_ROOT}/lib
)

# Find C library
find_library(PAHO_MQTT_C_LIBRARY
    NAMES paho-mqtt3a
    PATHS
        /usr/local/lib
        /usr/lib
        /usr/lib/aarch64-linux-gnu
        /usr/lib/arm-linux-gnueabihf
        ${CMAKE_SOURCE_DIR}/third_party/paho/lib
        $ENV{PAHO_MQTT_C_ROOT}/lib
)

# Find SSL version too
find_library(PAHO_MQTT_C_SSL_LIBRARY
    NAMES paho-mqtt3as
    PATHS
        /usr/local/lib
        /usr/lib
        /usr/lib/aarch64-linux-gnu
        /usr/lib/arm-linux-gnueabihf
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(PahoMqttCpp
    REQUIRED_VARS
        PAHO_MQTT_CPP_LIBRARY
        PAHO_MQTT_C_LIBRARY
        PAHO_MQTT_CPP_INCLUDE_DIR
        PAHO_MQTT_C_INCLUDE_DIR
)

if(PahoMqttCpp_FOUND AND NOT TARGET PahoMqttCpp::PahoMqttCpp)
    # Create the imported target
    add_library(PahoMqttCpp::PahoMqttCpp UNKNOWN IMPORTED)
    
    set_target_properties(PahoMqttCpp::PahoMqttCpp PROPERTIES
        IMPORTED_LOCATION "${PAHO_MQTT_CPP_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${PAHO_MQTT_CPP_INCLUDE_DIR};${PAHO_MQTT_C_INCLUDE_DIR}"
        INTERFACE_LINK_LIBRARIES "${PAHO_MQTT_C_LIBRARY}"
    )
    
    # Set additional properties
    target_link_libraries(PahoMqttCpp::PahoMqttCpp INTERFACE
        pthread
        ${OPENSSL_LIBRARIES}
    )
    
    message(STATUS "Found Paho MQTT C++: ${PAHO_MQTT_CPP_LIBRARY}")
    message(STATUS "Found Paho MQTT C: ${PAHO_MQTT_C_LIBRARY}")
endif()

mark_as_advanced(
    PAHO_MQTT_CPP_INCLUDE_DIR
    PAHO_MQTT_C_INCLUDE_DIR
    PAHO_MQTT_CPP_LIBRARY
    PAHO_MQTT_C_LIBRARY
    PAHO_MQTT_C_SSL_LIBRARY
)