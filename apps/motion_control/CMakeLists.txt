add_executable(motion_control main.cpp)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/inc)

#libraries
add_subdirectory ("hardware/encoders")
add_subdirectory ("hardware/imu")
add_subdirectory ("hardware/motors")

add_subdirectory ("apis/state_estimation")
add_subdirectory ("apis/publisher")
add_subdirectory ("apis/trajectory_follower")

#target_include_directories(motion_control PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/inc)

if(USE_RASPBERRY_PI)

    target_link_libraries(motion_control PRIVATE Imu State Publisher Encoders Motors Trajectory Threads::Threads
        gpiodcxx
        gpiod)

else()

    target_link_libraries(motion_control PRIVATE Imu State Publisher Encoders Motors Trajectory Threads::Threads)

endif()
