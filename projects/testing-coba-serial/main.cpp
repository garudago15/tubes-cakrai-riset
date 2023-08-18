/* mbed Microcontroller Library
 * Copyright (c) 2023 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mbed.h"
#include "../../KRAI_Library/TFMini/TFPlus.h"
#include "../../KRAI_Library/Pinout/F446RE_MASTER_2022.h"


/* UNTUK TF MINI */
// Pins and Configs
#define Serial_RX USBRX
#define Serial_TX USBTX
#define TFMP_SDA F446RE_MASTER_I2C_SDA
#define TFMP_SCL F446RE_MASTER_I2C_SCL

// Serial
static BufferedSerial serial_port(Serial_TX, Serial_RX, 115200);
FileHandle *mbed::mbed_override_console(int fd){
    return &serial_port;
}

// TFMini Plus Setup
const uint8_t TFMP_ADDR = 0x12;
TFplus TFMP(TFMP_SDA, TFMP_SCL, TFMP_ADDR);

int timer1 = us_ticker_read();
int timer2 = us_ticker_read();

/* UNTUK UART COM */
#define ESP_RX PA_10
#define ESP_TX PA_9

#define BLACK_RX PD_2
#define BLACK_TX PC_12

// Serial ESP32
BufferedSerial esp_uart(ESP_TX, ESP_RX, 115200);

// Serial ARCH_MAX
BufferedSerial arch_max_uart(BLACK_TX, BLACK_RX, 115200);

// Variables
float setpoint_rpm, current_rpm;
int setpoint_angle, current_angle;
float target_dist;

/* Debugin Purpose */
const int MAX_MSG = 64;
static char message[MAX_MSG];
static float shooter[2];
static int angle[2];
int constPID_pos = 0;
static unsigned int message_pos = 0;
char inByte;

float float1, float2;
int int1, int2;

// Calculate a simple checksum for a given string
uint8_t calculateChecksum(const char *data) {
    uint8_t checksum = 0;
    for (size_t i = 0; data[i] != '\0'; i++) {
        checksum ^= data[i];
    }
    return checksum;
}

int main()
{
    while(1){

        if(us_ticker_read()-timer1 > 1e6){
            TFMP.olahData();
            timer1 = us_ticker_read();
        }
        if(us_ticker_read()-timer2 > 1e6){
            printf("dist = %d, str = %d\n", TFMP.dist_HL, TFMP.strength_HL);
            timer2 = us_ticker_read();
        }


         while (arch_max_uart.readable())
        {
            arch_max_uart.read(&inByte, 1);

            if ((inByte != '\r') && message_pos < MAX_MSG - 1)
            {
                message[message_pos] = inByte;
                message_pos++;
            }
            else
            {
                message[message_pos] = '\0';
                message_pos = 0;
                char *token = strtok(message, " ");

                while (token != NULL)
                {
                    // kpKiKd[constPID_pos] = atof(token);
                    if(constPID_pos <= 1){
                        shooter[constPID_pos] = atof(token);
                    }
                    printf("%s %d\n", token, constPID_pos);
                    token = strtok(NULL, " ");
                    constPID_pos++;
                    if (constPID_pos == 3)
                    {
                        constPID_pos = 0;
                    }
                }

                // printf("kp = %f || ki = %f || kd = %f\n", pidLeftMotor.getPParam(), pidLeftMotor.getIParam(), pidLeftMotor.getDParam());
                // printf("kp = %f || ki = %f \n", shooter[0], shooter[1]);
            }
        }
    }

    return 0;
}