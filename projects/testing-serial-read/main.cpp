MASTER-MAGANG
/* mbed Microcontroller Library
 * Copyright (c) 2023 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mbed.h"
#include "../../KRAI_Library/TFMini/TFPlus.h"
#include "../../KRAI_Library/Pinout/F446RE_MASTER_2022.h"

// Pins and Configs
#define Serial_RX USBRX
#define Serial_TX USBTX

#define ESP_RX PA_10
#define ESP_TX PA_9

#define BLACK_RX PD_2
#define BLACK_TX PC_12

#define TFMP_SDA F446RE_MASTER_I2C_SDA
#define TFMP_SCL F446RE_MASTER_I2C_SCL

// Serial ESP32
BufferedSerial esp_uart(ESP_TX, ESP_RX, 115200);

// Serial ARCH_MAX
BufferedSerial arch_max_uart(BLACK_TX, BLACK_RX, 115200);

// Serial PC
static BufferedSerial serial_port(Serial_TX, Serial_RX, 115200);
FileHandle *mbed::mbed_override_console(int fd){
    return &serial_port;
}

// TFMini Plus Setup
const uint8_t TFMP_ADDR = 0x12;
TFplus TFMP(TFMP_SDA, TFMP_SCL, TFMP_ADDR);

// Variables
float setpoint_rpm, current_rpm;
int setpoint_angle, current_angle;
float target_dist;

int timer1 = us_ticker_read();
int timer2 = us_ticker_read();

/// Constants for serial
const int MAX_MSG = 64;
static char message[MAX_MSG];
int data_pos = 0;
static unsigned int message_pos = 0;
char inByte;
const char DELIMITER = ' ';

// Calculate a simple checksum for a given string
uint8_t calculateChecksum(const char *data) {
    uint8_t checksum = 0;
    for (size_t i = 0; data[i] != '\0'; i++) {
        checksum ^= data[i];
    }
    return checksum;
}


int main(){

    // Hanya dijalankan untuk mencari unknown address
    // TFMP.findAddress(); 

    while(1){

        if(us_ticker_read()-timer1 > 1e5){
            TFMP.olahData();
            timer1 = us_ticker_read();
        }

        // Baca data dari STM32 MAIN BOARD
        while (arch_max_uart.readable()) {
            arch_max_uart.read(&inByte, 1);

            if (inByte == DELIMITER) {
                message[message_pos] = '\0'; // Null-terminate the message

                // Calculate and compare the checksum
                uint8_t received_checksum = message[message_pos - 1];
                message[message_pos - 1] = '\0'; // Remove the checksum from the message
                uint8_t calculated_checksum = calculateChecksum(message);

                if (received_checksum == calculated_checksum) {

                    printf("Received valid data: %s\n", message);
                    
                    // Split the received message using the delimiter
                    char *token = strtok(message, " ");
                    current_rpm = atof(token);
                    token = strtok(NULL, " ");
                    setpoint_rpm = atoi(token);
                    token = strtok(NULL, " ");
                    current_angle = atof(token);
                    token = strtok(NULL, " ");
                    setpoint_angle = atoi(token);

                    // Send the string over UART
                    // Convert the values to a string
                    char msg[50];  // Adjust the size as needed
                    

                    // Send the string over UART to esp_uart
                    // printf("%s", msga);
                    esp_uart.write(msg, strlen(msg));

                } else {
                    printf("Corrupted data: %s\n", message);
                }

                // Reset message position
                message_pos = 0;
            } else {

                // Store character in the message buffer
                message[message_pos] = inByte;
                message_pos++;

            }

        }

        if(us_ticker_read()-timer2 > 1e5){

            // Jarak TFMini + (Jarak Penembak -> TFMini)
            target_dist = ((float) TFMP.dist_HL  / 100.0f) + 0.55f; // Jarak TFMini (Meter)

            timer2 = us_ticker_read();
        }
    
        // Send to ARCH_MAX
        // printf("%f %d\n", rpm, angleInDegrees );

        snprintf(msg, sizeof(msg), "%f %f %f %d %d\n", target_dist, current_rpm, setpoint_rpm, current_angle, setpoint_angle);

    }
}
