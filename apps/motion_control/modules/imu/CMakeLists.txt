if(USE_RASPBERRY_PI)

add_library(Imu STATIC 
            imu.cpp
            )
else()
add_library(Imu STATIC 
            imu_mock.cpp
            )
endif()

# target_sources(robot PRIVATE 
# "${CMAKE_CURRENT_SOURCE_DIR}/encoders.cpp"
# )

target_include_directories(Imu PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/..)