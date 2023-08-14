/* mbed Microcontroller Library
 * Copyright (c) 2023 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mbed.h"
#include "../../libs/led4Pin/led4Pin.h"

// DEFINE LED
#define red PA_10
#define green PB_4
#define blue PB_10
LED4 RGB(red, green, blue);

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