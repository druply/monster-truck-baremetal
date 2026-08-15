if(USE_RASPBERRY_PI)

add_library(Publisher STATIC 
            Publisher.cpp
            )
else()
add_library(Publisher STATIC 
            Publisher_mock.cpp
            )
endif()

# target_sources(robot PRIVATE 
# "${CMAKE_CURRENT_SOURCE_DIR}/encoders.cpp"
# )

target_include_directories(Publisher PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../..)
target_include_directories(Publisher PRIVATE ${CMAKE_SOURCE_DIR})
target_link_libraries(Publisher PRIVATE Imu, Motors, Encoders)