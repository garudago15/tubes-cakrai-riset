/* mbed Microcontroller Library
 * Copyright (c) 2023 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mbed.h"
#include "../../KRAI_Library/encoderHAL/encoderHAL.h"
#include "../../KRAI_Library/encoderHAL/encoderMspInitF4.h"
#include "../../KRAI_Library/pinout/F407VET6_2023.h"
#include "../../KRAI_Library/encoderKRAI/encoderKRAI.h"


#define PPR_CUI 100

static BufferedSerial serial_port(PA_9, PA_10, 115200);
FileHandle *mbed::mbed_override_console(int fd)
{
    return &serial_port;
}

// encoderHAL enc(F407VET6_ENCODER_2_2_A, F407VET6_ENCODER_2_2_B, PPR_CUI, Encoding::X4_ENCODING);
encoderHAL enc(TIM4);
HAL_TIM_Encoder_MspInit(enc.getTimer());

const float kel = 2 * 3.14f * 2.9f;
float pulse, rotation;
float s;

int millis_ms()
{
    return (us_ticker_read() / 1000);
}

int main()
{   
    while (true)
    {
        // Coba
        pulse = (float)(enc.getPulses());
        rotation = pulse / (PPR_CUI * 4);
        s = rotation * kel;
        printf("PULSE: %d; DISTANCE: %f cm\n", enc.getPulses(), s);
    }
}