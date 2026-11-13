cmake_minimum_required(VERSION 3.20)
project(AnimalTruck LANGUAGES CXX)

option(ENABLE_ASAN "Build with AddressSanitizer" OFF)
option(ENABLE_TSAN "Build with ThreadSanitizer" OFF)
option(ENABLE_HARDENING "Enable compiler hardening options" ON)
option(ENABLE_STRICT_WARNINGS "Enable strict Clang warnings and treat them as errors" ON)

if(ENABLE_ASAN AND ENABLE_TSAN)
    message(FATAL_ERROR "ENABLE_ASAN and ENABLE_TSAN cannot be enabled together.")
endif()

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

find_package(Threads REQUIRED)

add_library(project_build_options INTERFACE)

if(ENABLE_STRICT_WARNINGS AND CMAKE_CXX_COMPILER_ID MATCHES "^(Clang|AppleClang)$")
    target_compile_options(project_build_options INTERFACE
        -Wall
        -Wextra
        -Wpedantic
        -Werror
        -Wconversion
        -Wsign-conversion
        -Wshadow
        -Wold-style-cast
        -Woverloaded-virtual
        -Wnull-dereference
        -Wdouble-promotion
        -Wformat=2
        -Wundef
    )
endif()

if(ENABLE_HARDENING)
    if(CMAKE_CXX_COMPILER_ID MATCHES "^(GNU|Clang|AppleClang)$")
        target_compile_options(project_build_options INTERFACE
            -fstack-protector-strong
            "$<$<OR:$<CONFIG:Release>,$<CONFIG:RelWithDebInfo>,$<CONFIG:MinSizeRel>>:-D_FORTIFY_SOURCE=2>"
        )
    elseif(MSVC)
        target_compile_options(project_build_options INTERFACE /GS /sdl)
    endif()
endif()

if(ENABLE_ASAN OR ENABLE_TSAN)
    if(CMAKE_CXX_COMPILER_ID MATCHES "^(GNU|Clang|AppleClang)$")
        if(ENABLE_ASAN)
            set(SANITIZER_FLAG -fsanitize=address)
        else()
            set(SANITIZER_FLAG -fsanitize=thread)
        endif()
        target_compile_options(project_build_options INTERFACE
            ${SANITIZER_FLAG}
            -fno-omit-frame-pointer
            -g
        )
        target_link_options(project_build_options INTERFACE ${SANITIZER_FLAG})
    elseif(MSVC AND ENABLE_ASAN)
        target_compile_options(project_build_options INTERFACE /fsanitize=address)
        target_link_options(project_build_options INTERFACE /fsanitize=address)
    else()
        message(FATAL_ERROR
            "The requested sanitizer is not supported with ${CMAKE_CXX_COMPILER_ID}.")
    endif()
endif()

add_library(robot_core
    src/core/odometry.cpp
)
target_include_directories(robot_core PUBLIC include)
target_link_libraries(robot_core PUBLIC project_build_options)

add_library(robot_runtime
    src/app/robot.cpp
    src/app/robot_factory.cpp
)
target_include_directories(robot_runtime PUBLIC include)
target_link_libraries(robot_runtime PUBLIC robot_core project_build_options)

add_library(robot_pca9685
    src/drivers/pwm.cpp
)
target_include_directories(robot_pca9685 PUBLIC include)
target_link_libraries(robot_pca9685 PUBLIC project_build_options Threads::Threads)

add_executable(Robot src/main.cpp)
target_include_directories(Robot PRIVATE test)
target_link_libraries(Robot PRIVATE
    robot_runtime
    robot_pca9685
    project_build_options
    Threads::Threads
)

include(CTest)
if(BUILD_TESTING)
    add_executable(robot_tests test/robot_runtime_test.cpp)
    target_include_directories(robot_tests PRIVATE test)
    target_link_libraries(robot_tests PRIVATE robot_runtime robot_pca9685 project_build_options)
    add_test(NAME robot_tests COMMAND robot_tests)
endif()
