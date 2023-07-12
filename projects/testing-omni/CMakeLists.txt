# Copyright (c) 2023 ARM Limited. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.19.0)

set(MBED_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../../mbed-os CACHE INTERNAL "")
set(MBED_CONFIG_PATH ${CMAKE_CURRENT_BINARY_DIR} CACHE INTERNAL "")
set(APP_TARGET testing-omni)

include(${MBED_PATH}/tools/cmake/app.cmake)

project(${APP_TARGET})

add_subdirectory(${MBED_PATH} ${CMAKE_CURRENT_BINARY_DIR}/mbed-os)

add_executable(${APP_TARGET}
    main.cpp
    ../../libs/JoystickPS3/JoystickPS3.cpp
    ../../libs/Control4Roda/Control4Omni.cpp
    ../../libs/Control4Roda/Control4Roda.cpp
    ../../libs/odom3enc/odom3enc.cpp
    ../../libs/Motor/Motor.cpp
    ../../libs/ControlMotor/ControlMotor.cpp
    ../../libs/encoderHAL/encoderHAL.cpp
    ../../libs/encoderKRAI/encoderKRAI.cpp
    ../../libs/StanleyPursuit/StanleyPursuit.cpp
    ../../libs/pidLo/pidLo.cpp
    ../../libs/SMC_KRAI/SMC_KRAI.cpp
)

target_link_libraries(${APP_TARGET} mbed-os)

mbed_set_post_build(${APP_TARGET})

option(VERBOSE_BUILD "Have a verbose build process")
if(VERBOSE_BUILD)
    set(CMAKE_VERBOSE_MAKEFILE ON)
endif()