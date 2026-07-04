add_executable(motion_control main.cpp)

#libraries
add_subdirectory ("modules/encoders")
add_subdirectory ("modules/imu")
add_subdirectory ("modules/motors")

add_subdirectory ("apis/state_estimation")
add_subdirectory ("apis/trajectory_follower")

if(USE_RASPBERRY_PI)

    target_link_libraries(motion_control PRIVATE Imu  State Encoders Motors Trajectory Threads::Threads
        gpiodcxx
        gpiod)

else()

    target_link_libraries(motion_control PRIVATE  Imu State Encoders Motors Trajectory Threads::Threads)

endif()
