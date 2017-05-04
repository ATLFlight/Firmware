cmake_minimum_required(VERSION 2.8.12)
project("raspberry_px4")

# TODO-JYW: TESTING-TESTING
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} /usr/local/lib/fastrtps/cmake:/usr/local/lib/fastcdr/cmake)

# Find requirements
find_package(fastrtps REQUIRED)
find_package(fastcdr REQUIRED)

# TODO-JYW: TESTING-TESTING
set(fastcdr_INCLUDE_DIR "/usr/local/include/fastrtps")


# Set C++11
include(CheckCXXCompilerFlag)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_COMPILER_IS_CLANG OR
        CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    check_cxx_compiler_flag(--std=c++11 SUPPORTS_CXX11)
    if(SUPPORTS_CXX11)
        add_compile_options(--std=c++11)
    else()
        message(FATAL_ERROR "Compiler doesn't support C++11")
    endif()
endif()

file(GLOB RASPBERRY_PX4_SOURCES "*.cxx")

add_executable(raspberry_px4 ${RASPBERRY_PX4_SOURCES})
target_link_libraries(raspberry_px4 fastrtps fastcdr)
