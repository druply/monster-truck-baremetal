if(USE_RASPBERRY_PI)

add_library(Encoders STATIC 
            encoders.cpp
            )
else()
add_library(Encoders STATIC 
            encoders_mock.cpp
            )
endif()

# target_sources(robot PRIVATE 
# "${CMAKE_CURRENT_SOURCE_DIR}/encoders.cpp"
# )

target_include_directories(Encoders PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/..)