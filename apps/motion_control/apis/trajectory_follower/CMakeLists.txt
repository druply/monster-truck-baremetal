if(USE_RASPBERRY_PI)

add_library(Trajectory STATIC 
            trajectory_follower.cpp
            )
else()
add_library(Trajectory STATIC 
            trajectory_follower.cpp
            )
endif()

# target_sources(robot PRIVATE 
# "${CMAKE_CURRENT_SOURCE_DIR}/encoders.cpp"
# )

target_include_directories(Trajectory PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../..)
target_include_directories(Trajectory PRIVATE ${CMAKE_SOURCE_DIR})
#target_link_libraries(Trajectory PRIVATE Imu)