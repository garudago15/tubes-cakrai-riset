/* mbed Microcontroller Library
 * Copyright (c) 2023 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mbed.h"
// #include "../KRAI_Library/Motor/Motor.h"
#include "../KRAI_Library/encoderKRAI/encoderKRAI.h"
// #include "../KRAI_Library/pidLo/pidLo.h" // Include the pidLo library
// #include "../KRAI_Library/Pinout/F407VET6_2023.h"

#include "../libs/Configs/Constants.h"
// #include "../libs/Configs/ConfigurationPin.h"
#include "../KRAI_Library/Pinout/F446RE_MASTER_2022.h"
#include "../libs/MovingAverage/MovingAverage.h"
#include "../libs/noPWMMotor/noPWMMotor.h"

/* INITIALIZE SERIAL USING PRINTF */
static BufferedSerial serial_port(USBTX, USBRX, 115200);
FileHandle *mbed::mbed_override_console(int fd){
    return &serial_port;
}

// Motor mot(F446RE_MASTER_PWM_MOTOR_C, F446RE_MASTER_FOR_MOTOR_C, F446RE_MASTER_REV_MOTOR_C);
noPWMMotor motor(F446RE_MASTER_PWM_MOTOR_C, F446RE_MASTER_FOR_MOTOR_C, F446RE_MASTER_REV_MOTOR_C);
encoderKRAI enc(F446RE_MASTER_ENCODER_A_D_A, F446RE_MASTER_ENCODER_A_D_B, ENC_BASE_MOTOR_PPR / 2, X2_ENCODING);
MovingAverage Avg(20);

float lastRPM=0;
uint32_t t0=0, t1=0;
#define virtualPWMPeriod 500 //in us
#define printPeriod 500000
#define PPR 538
#define absoluteMaxAcc 1.0f
float setpoint=120.0f;
float toRPMMultiplier= 60.0f * 1000000.0f / (float)virtualPWMPeriod / (float)PPR;
float toAccRPMMultiplier = 1000000.0f / (float)virtualPWMPeriod;

float speedRPM, realSpeedRPM, accRPM, maxAccRPM;
int32_t tmpPulse = enc.getPulses();
float tmpSpeedRPM=0.0f;

float  output;
float motor_default_speed=0.5;

int main()
{
    while(true){
        // if(us_ticker_read()-t0>virtualPWMPeriod){
        //     //get current speed
        //     realSpeedRPM = (float)(enc.getPulses() - tmpPulse) * toRPMMultiplier;
        //     speedRPM = Avg.movingAverage(realSpeedRPM);
        //     accRPM = (speedRPM - tmpSpeedRPM) * toAccRPMMultiplier;

        //     //taking tmp values
        //     tmpPulse=enc.getPulses();
        //     tmpSpeedRPM=speedRPM;

        //     maxAccRPM=absoluteMaxAcc;

        //     if(accRPM>maxAccRPM){ //max acceleration check
        //         motor.onOff(0);
        //     } else if(speedRPM>setpoint){ //max speed check
        //         motor.onOff(0);
        //     } else{
        //         motor.onOff(1);
        //     }

        //     t0=us_ticker_read();
        // }

        if(us_ticker_read()-t1>printPeriod){
            motor.onOff(1);
            printf("enc: %d\n",enc.getPulses());
            t1=us_ticker_read();
        }
        // printf("%d\n",enc.getPulses());
    }
    return 0;
}