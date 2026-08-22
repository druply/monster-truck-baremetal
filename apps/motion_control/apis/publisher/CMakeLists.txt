if(USE_RASPBERRY_PI)

add_library(Publisher STATIC 
            Publisher.cpp
            )
else()
add_library(Publisher STATIC 
            Publisher.cpp
            )
endif()

# target_sources(robot PRIVATE 
# "${CMAKE_CURRENT_SOURCE_DIR}/encoders.cpp"
# )

target_include_directories(Publisher PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../..)
target_include_directories(Publisher PRIVATE ${CMAKE_SOURCE_DIR})
    # PUBLIC 
    # PahoMqttCpp::PahoMqttCpp    # C++ wrapper
    # ${PAHO_MQTT_C_LIB}          # C library (async)
    # ${PAHO_MQTT_C_SSL_LIB}      # C SSL library (optional)

target_link_libraries(Publisher 
    PRIVATE
    paho-mqttpp3     # C++ wrapper (must be first)
    paho-mqtt3a      # C async library
    pthread          # Threading
    ssl              # SSL support (if using SSL)
    crypto           # Crypto library (if using SSL)
    boost_json
)