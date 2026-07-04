if(USE_RASPBERRY_PI)

add_library(State STATIC 
            state_estimation.cpp
            )
else()
add_library(State STATIC 
            state_mock.cpp
            )
endif()

# target_sources(robot PRIVATE 
# "${CMAKE_CURRENT_SOURCE_DIR}/encoders.cpp"
# )

target_include_directories(State PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../..)
target_include_directories(State PRIVATE ${CMAKE_SOURCE_DIR})
target_link_libraries(State PRIVATE Imu)