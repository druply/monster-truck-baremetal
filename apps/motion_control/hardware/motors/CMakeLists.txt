if(USE_RASPBERRY_PI)

add_library(Motors STATIC 
            motors.cpp
            PWM.cpp
            )
else()
add_library(Motors STATIC 
            motors.cpp
            PWM_mock.cpp
            )
endif()

# target_sources(robot PRIVATE 
# "${CMAKE_CURRENT_SOURCE_DIR}/encoders.cpp"
# )

target_include_directories(Motors PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/..)