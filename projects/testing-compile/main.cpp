/* mbed Microcontroller Library
 * Copyright (c) 2023 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mbed.h"
DigitalOut led1(LED1);
int counting=0;


static BufferedSerial serial_port(PA_9, PA_10, 115200);
FileHandle *mbed::mbed_override_console(int fd){
    return &serial_port;
}

int timer1=us_ticker_read();
int now=us_ticker_read();

int main()
{
    led1=0;
    while(1){
        now=us_ticker_read();
        if(now-timer1>500000){
            led1!=led1;
            printf("Hello, %d\n", counting);
            counting++;
            timer1=now;
        }
    }
    printf("Hello, Mbed!\n");
    return 0;
}