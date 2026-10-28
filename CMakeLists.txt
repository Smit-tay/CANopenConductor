cmake_minimum_required(VERSION 3.16)
project(CANopenConductor VERSION 0.1.0 LANGUAGES CXX)

option(CANOPEN_CONDUCTOR_WERROR "Treat warnings as errors" OFF)

# Library-private flags. Never propagated to consumers.
function(canopen_conductor_private_options target)
    if(MSVC)
        target_compile_options(${target} PRIVATE /W4)
        if(CANOPEN_CONDUCTOR_WERROR)
            target_compile_options(${target} PRIVATE /WX)
        endif()
    else()
        target_compile_options(${target} PRIVATE
            -Wall -Wextra -Wpedantic -Wconversion -Wshadow
            -fno-exceptions -fno-rtti)
        if(CANOPEN_CONDUCTOR_WERROR)
            target_compile_options(${target} PRIVATE -Werror)
        endif()
    endif()
endfunction()

# One library. The protocol sources use no OS headers. The connection sources (link, slcan, SocketCAN) are
# compiled per platform: the serial port is POSIX or Windows, SocketCAN is Linux only.
if(WIN32)
    set(_conductor_platform src/serial_win32.cpp)
elseif(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    set(_conductor_platform src/serial_posix.cpp src/socketcan_bus.cpp)
else()
    set(_conductor_platform src/serial_posix.cpp)
endif()

add_library(canopen_conductor
    src/lss_master.cpp
    src/master.cpp
    src/nmt_master.cpp
    src/node_table.cpp
    src/pdo.cpp
    src/sdo_client.cpp
    src/sync_producer.cpp
    src/link.cpp
    src/session.cpp
    src/slcan_bus.cpp
    src/slcan_codec.cpp
    src/waker.cpp
    ${_conductor_platform}
)
add_library(CANopenConductor::CANopenConductor ALIAS canopen_conductor)
target_include_directories(canopen_conductor PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_compile_features(canopen_conductor PUBLIC cxx_std_20)
# Usable inside a shared library or a plug-in module
set_target_properties(canopen_conductor PROPERTIES POSITION_INDEPENDENT_CODE ON)
canopen_conductor_private_options(canopen_conductor)

# Tests of the connection (Linux: a pty plays the slcan adapter).
option(CANOPEN_CONDUCTOR_TESTS "Build the connection tests" OFF)
if(CANOPEN_CONDUCTOR_TESTS AND CMAKE_SYSTEM_NAME STREQUAL "Linux")
    enable_testing()
    foreach(_test link_test session_test)
        add_executable(${_test} test/${_test}.cpp)
        target_link_libraries(${_test} PRIVATE canopen_conductor)
        add_test(NAME ${_test} COMMAND ${_test})
    endforeach()
endif()

# The console: a small program to try the connection by hand, on real hardware.
option(CANOPEN_CONDUCTOR_CONSOLE "Build canopen-console" OFF)
if(CANOPEN_CONDUCTOR_CONSOLE)
    add_executable(canopen-console console/main.cpp console/terminal.cpp)
    target_link_libraries(canopen-console PRIVATE canopen_conductor)
    find_package(Threads REQUIRED)
    target_link_libraries(canopen-console PRIVATE Threads::Threads)
endif()
