# CMakeList.txt : CMake project for LogFile, include source and define
# project specific logic here.
#
cmake_minimum_required (VERSION 3.13)

# 1. Add a manual option to override detection if needed
option(USE_RASPBERRY_PI "Build for Raspberry Pi hardware" OFF)

project ("AnimalTruck")

option(ENABLE_ASAN "Build with AddressSanitizer" ON)
option(ENABLE_TSAN "Build with ThreadSanitizer" ON)
option(ENABLE_HARDENING "Enable compiler hardening options" ON)
option(ENABLE_STRICT_WARNINGS "Enable strict Clang warnings and treat them as errors" ON)

if(ENABLE_ASAN AND ENABLE_TSAN)
    message(FATAL_ERROR "ENABLE_ASAN and ENABLE_TSAN cannot be enabled together.")
endif()

message(STATUS "C++ Compiler ID: ${CMAKE_CXX_COMPILER_ID}")
message(STATUS "C++ Compiler Version: ${CMAKE_CXX_COMPILER_VERSION}")
message(STATUS "C++ Compiler Path: ${CMAKE_CXX_COMPILER}")

# Find the required system threads library (pthread)
find_package(Threads REQUIRED)

#set(BOOST_ROOT "C:\\boost_libraries") 
#set(Boost_USE_STATIC_LIBS ON)
#set(BOOST_LIBRARYDIR "${BOOST_ROOT}/stage/lib")
#set(Boost_DEBUG ON) 
# find_package(Boost)

if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    message(STATUS "Using GCC compiler, adding specific flags.")
    # Add GCC specific options
    set(BOOST_ROOT /usr/local)
    set(Boost_INCLUDE_DIR /usr/local/include)
    set(Boost_LIBRARY_DIR /usr/local/lib)

    if(Boost_FOUND)
    include_directories(${Boost_INCLUDE_DIRS})
    link_directories(${Boost_LIBRARY_DIRS})
    endif()

    # Add custom CMake modules path
    #list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
    find_library(PAHO_CPP_LIB paho-mqttpp3)
    find_library(PAHO_C_LIB paho-mqtt3a)


    if(WIN32)
	    include_directories("C:\\Poco\\include")
        include_directories("C:\\boost_libraries")
	    set(Poco_DIR "C:\\Poco\\cmake") 
    endif()
elseif (CMAKE_CXX_COMPILER_ID STREQUAL "Clang" OR CMAKE_CXX_COMPILER_ID STREQUAL "AppleClang")
    message(STATUS "Using Clang compiler, adding specific flags.")
    # Add Clang specific options
    if(APPLE)
      message(STATUS "Building on macOS or other Apple platform")
      find_package(Boost REQUIRED)
    endif()

elseif (CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    message(STATUS "Using Visual Studio C++ compiler.")
    # Add MSVC specific options
    if(WIN32)
	    include_directories("C:\\PocoVS\\include")
        include_directories("C:\\boost_libraries")
	    set(Poco_DIR "C:\\PocoVS\\cmake") 
     endif()
endif()


include_directories("include")
include_directories("test")

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED true)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

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

#db2 library header files
#include_directories(${DB2_INCLUDE_PATH}/include)


#include_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib/LogFile)
#include_directories(${CMAKE_CURRENT_SOURCE_DIR}/utils/Logger)
#include_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib/database)

add_executable(Robot src/main.cpp src/robot.cpp
    src/controllers/odometry.cpp)
target_link_libraries(Robot PRIVATE project_build_options)

#libraries
#add_subdirectory ("apps/motion_control")
#add_subdirectory ("modules/imu")
#add_subdirectory ("modules/motors")

#target_link_libraries(Robot PRIVATE Encoders Motors IMU)
