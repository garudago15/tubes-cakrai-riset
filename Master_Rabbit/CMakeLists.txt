# Copyright (c) 2022 ARM Limited. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.19.0)

set(MBED_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../mbed-os CACHE INTERNAL "")
set(MBED_CONFIG_PATH ${CMAKE_CURRENT_BINARY_DIR} CACHE INTERNAL "")
set(APP_TARGET Master_Rabbit)

include(${MBED_PATH}/tools/cmake/app.cmake)

project(${APP_TARGET})

add_subdirectory(${MBED_PATH} ${CMAKE_CURRENT_BINARY_DIR}/mbed-os)

add_executable(${APP_TARGET}
    ../KRAI_Library/CMPS12_KRAI/CMPS12_KRAI.cpp
    ../KRAI_Library/pidLo/pidLo.cpp
    ../KRAI_Library/SMC_KRAI/SMC_KRAI.cpp
    ../KRAI_Library/ControlMotor/ControlMotor.cpp
    ../KRAI_Library/Motor/Motor.cpp
    ../KRAI_Library/encoderHAL/encoderHAL.cpp
    ../KRAI_Library/encoderKRAI/encoderKRAI.cpp
    ../KRAI_Library/odom3enc/odom3enc.cpp
    ../KRAI_Library/odom2enc/odom2enc.cpp
    ../KRAI_Library/JoystickPS3/JoystickPS3.cpp
    ../KRAI_Library/StanleyPursuit/StanleyPursuit.cpp
    ../KRAI_Library/Control4Roda/Control4Roda.cpp
    ../KRAI_Library/Control4Roda/Control4Mechanum.cpp
    ../KRAI_Library/Control4Roda/ControlAutomatic4Mechanum.cpp
    ../KRAI_Library/ultrasonicKRAI/ultrasonicKRAI.cpp
    ../KRAI_Library/TFMini/TFPlus.cpp
    ../KRAI_Library/PIDAaronBerk/PIDAaronBerk.cpp
    main.cpp
)

target_link_libraries(${APP_TARGET} mbed-os)

mbed_set_post_build(${APP_TARGET})

option(VERBOSE_BUILD "Have a verbose build process")
if(VERBOSE_BUILD)
    set(CMAKE_VERBOSE_MAKEFILE ON)
endif()