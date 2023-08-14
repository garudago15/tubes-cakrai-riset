/* mbed Microcontroller Library
 * Copyright (c) 2023 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mbed.h"
#include "../../libs/led4Pin/led4Pin.h"

// DEFINE LED
#define red PA_5
#define green PA_6
#define blue PA_7
LED4Pin RGB(red, green, blue);

int main()
{
    while (true)
    {
        
        RGB.setColor("RED");
        wait_us(1000000);
        RGB.setColor("GREEN");
        wait_us(1000000);
        RGB.setColor("BLUE");
        wait_us(1000000);
        RGB.setColor("PURPLE");
        wait_us(1000000);
        RGB.setColor("CYAN");
        wait_us(1000000);
        RGB.setColor("YELLOW");
        wait_us(1000000);
        RGB.setColor("ORANGE");
        wait_us(1000000);

    }
    
    return 0;
}