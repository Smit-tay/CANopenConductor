cmake_minimum_required(VERSION 3.16)
project(CANopenConductor VERSION 0.1.0 LANGUAGES CXX)

if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    set(_conductor_socketcan_default ON)
else()
    set(_conductor_socketcan_default OFF)
endif()

option(CANOPEN_CONDUCTOR_SOCKETCAN "Build the SocketCAN bus (Linux)" ${_conductor_socketcan_default})
option(CANOPEN_CONDUCTOR_SLCAN "Build the slcan serial bus (POSIX and Windows)" ON)
option(CANOPEN_CONDUCTOR_WERROR "Treat warnings as errors" OFF)

# Library-private flags. Never propagated to consumers.
function(canopen_conductor_private_options target)
    if(MSVC)
        target_compile_options(${target} PRIVATE /W4 /EHs-c- /GR-)
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

# Core library: no threads, no I/O, no heap use after create().
add_library(canopen_conductor
    src/lss_master.cpp
    src/master.cpp
    src/nmt_master.cpp
    src/node_table.cpp
    src/pdo.cpp
    src/sdo_client.cpp
    src/sync_producer.cpp
)
add_library(CANopenConductor::CANopenConductor ALIAS canopen_conductor)
target_include_directories(canopen_conductor PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_compile_features(canopen_conductor PUBLIC cxx_std_20)
canopen_conductor_private_options(canopen_conductor)

# SocketCAN bus (Linux).
if(CANOPEN_CONDUCTOR_SOCKETCAN)
    add_library(canopen_conductor_socketcan ports/linux/socketcan_bus.cpp)
    add_library(CANopenConductor::SocketCAN ALIAS canopen_conductor_socketcan)
    target_include_directories(canopen_conductor_socketcan PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/ports/linux>)
    target_link_libraries(canopen_conductor_socketcan PUBLIC canopen_conductor)
    canopen_conductor_private_options(canopen_conductor_socketcan)
endif()

# slcan serial bus (POSIX tty or Windows COM port).
if(CANOPEN_CONDUCTOR_SLCAN)
    if(WIN32)
        set(_conductor_serial ports/slcan/serial_win32.cpp)
    else()
        set(_conductor_serial ports/slcan/serial_posix.cpp)
    endif()
    add_library(canopen_conductor_slcan
        ports/slcan/slcan_bus.cpp
        ports/slcan/slcan_codec.cpp
        ${_conductor_serial})
    add_library(CANopenConductor::Slcan ALIAS canopen_conductor_slcan)
    target_include_directories(canopen_conductor_slcan PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/ports/slcan>)
    target_link_libraries(canopen_conductor_slcan PUBLIC canopen_conductor)
    canopen_conductor_private_options(canopen_conductor_slcan)
endif()
