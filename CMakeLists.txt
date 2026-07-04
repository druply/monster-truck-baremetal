# CMakeList.txt : CMake project for LogFile, include source and define
# project specific logic here.
#
cmake_minimum_required (VERSION 3.8)

# 1. Add a manual option to override detection if needed
option(USE_RASPBERRY_PI "Build for Raspberry Pi hardware" OFF)

project ("AnimalTruck")

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

# if(WIN32)
# find_package(ODBC REQUIRED)
# endif()

# # Find the POCO package and the necessary components (e.g., Foundation, Util, Net)
# find_package(Poco REQUIRED Foundation Util Net DataODBC Data)

include_directories("include")

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED true)

#set(DB2_INCLUDE_PATH "/opt/ibm/db2/V12.1")
#link_directories(${DB2_INCLUDE_PATH}/lib64)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

#db2 library header files
#include_directories(${DB2_INCLUDE_PATH}/include)


#include_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib/LogFile)
#include_directories(${CMAKE_CURRENT_SOURCE_DIR}/utils/Logger)
#include_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib/database)

#add_executable(Robot main.cpp)

#libraries
add_subdirectory ("apps/motion_control")
#add_subdirectory ("modules/imu")
#add_subdirectory ("modules/motors")

#target_link_libraries(Robot PRIVATE Encoders)
