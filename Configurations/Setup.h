#pragma once

#include "mbed.h"
#include "../../KRAI_Library/encoderKRAI/encoderKRAI.h"
#include "../../KRAI_Library/Motor/Motor.h"
#include "../../KRAI_Library/PIDAaronBerk/PIDAaronBerk.h"
#include "../../KRAI_Library/MovingAverage/MovingAverage.h"
#include "Constants.h"

/* Time Sampling
    1.000.000 mikroSecond = 1 second
    1.000     mikroSecond = 1 miliSecond
*/
uint32_t samplingPID = 10 * 1000; // Mikro Second

// PID (Pernah motor pg45, tanpa beban, berhasil dituning dengan konstanta berikut)
float kp_LM = 0.75f;
float ki_LM = 0.5f;
float kd_LM = 0.0038f;
PIDAaronBerk pidLeftMotor(kp_LM, ki_LM, kd_LM, samplingPID/1000000.0f);

float kp_RM = 0.0f;
float ki_RM = 0.0f;
float kd_RM = 0.0f;
PIDAaronBerk pidRightMotor(kp_RM, ki_RM, kd_RM, samplingPID/1000000.0f);

// Encoder and Motor Shooter
Motor leftMotor(PWM_LM, FOR_LM, REV_LM);
Motor rightMotor(PWM_RM, FOR_RM, REV_RM);
encoderKRAI encLeftMotor(CHB_LM, CHA_LM, PPR_LM, Encoding::X4_ENCODING); // Dibalik dulu karena phase A dan B kabelnya... :)
encoderKRAI encRightMotor(CHA_RM, CHB_RM, PPR_RM, Encoding::X4_ENCODING);

// Moving Average
MovingAverage movAvgLM(20);
MovingAverage movAvgRM(20);

// Variabel Inisialisasi
float timeLast = us_ticker_read();

// INITIALIZE SERIAL USING PRINTF 
static BufferedSerial serial_port(USBTX, USBRX, 115200);
FileHandle *mbed::mbed_override_console(int fd){
    return &serial_port;
}