/* mbed Microcontroller Library
 * Copyright (c) 2023 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mbed.h"
#include "../../KRAI_Library/Motor/Motor.h"
#include "../../KRAI_Library/encoderKRAI/encoderKRAI.h"
// #include "../../KRAI_Library/pidLo/pidLo.h" // Include the pidLo library
// #include "../../KRAI_Library/Pinout/F407VET6_2023.h"

#include "../../libs/Configs/Constants.h"
// #include "../../libs/Configs/ConfigurationPin.h"
#include "../../KRAI_Library/Pinout/F446RE_MASTER_2022.h"
#include "../../libs/MovingAverage/MovingAverage.h"
#include "../../libs/noPWMMotor/noPWMMotor.h"
#include "../../libs/encoderMotorSpeed/encoderMotorSpeed.h"

/* INITIALIZE SERIAL USING PRINTF */
static BufferedSerial serial_port(USBTX, USBRX, 115200);
FileHandle *mbed::mbed_override_console(int fd){
    return &serial_port;
}

Motor mot(F446RE_MASTER_PWM_MOTOR_C, F446RE_MASTER_FOR_MOTOR_C, F446RE_MASTER_REV_MOTOR_C);
noPWMMotor motor(F446RE_MASTER_PWM_MOTOR_C, F446RE_MASTER_FOR_MOTOR_C, F446RE_MASTER_REV_MOTOR_C);
encoderKRAI enc(F446RE_MASTER_ENCODER_A_D_A, F446RE_MASTER_ENCODER_A_D_B, ENC_BASE_MOTOR_PPR / 2, X2_ENCODING);
encoderMotorSpeed encM(F446RE_MASTER_ENCODER_A_D_A, F446RE_MASTER_ENCODER_A_D_B, ENC_BASE_MOTOR_PPR / 2, X2_ENCODING);
MovingAverage Avg(20);

float lastRPM=0;
uint32_t t0=0, t1=0, t2=0;
#define virtualPWMPeriod 500 //in us
#define printPeriod 500000
#define PPR 538
#define absoluteMaxAcc 1.0f
#define TIMEOUT 250
#define baseSpeed 120.0f
uint32_t accelPeriod = 1000; //in ms
float setpoint=120.0f;
float tmpSetpoint=0.0f;
float toRPMMultiplier= 60.0f * 1000000.0f / (float)virtualPWMPeriod / (float)PPR;
float toRPMMultiplierM = 60.0f * 1000000.0f / (float)PPR;
float toAccRPMMultiplier = 1000000.0f / (float)virtualPWMPeriod;

float speedRPM, realSpeedRPM, accRPM, maxAccRPM;
float speedRPM_, speedRPMM;
int32_t tmpPulse = enc.getPulses();
float tmpSpeedRPM=0.0f;

float  output;
float motor_default_speed=0.5;

float accelResolution=100.0f;

float getAvgSpeed(){
    realSpeedRPM = (float)(enc.getPulses() - tmpPulse) * toRPMMultiplier;
    tmpPulse=enc.getPulses();
    return Avg.movingAverage(realSpeedRPM);
}

float getAvgSpeedM(){
    if(us_ticker_read() - encM.prevt > TIMEOUT){
        return 0.0f;
    } else{
        return toRPMMultiplierM/(float)encM.delta;
    }
}

Ticker virtualPWMtick;
void virtualPWM(){
    speedRPM_ = getAvgSpeed();
    speedRPMM = getAvgSpeedM();

    speedRPM = speedRPM_; //change this
    // accRPM = (speedRPM - tmpSpeedRPM) * toAccRPMMultiplier;

    // //taking tmp values
    
    // tmpSpeedRPM=speedRPM;

    // maxAccRPM=absoluteMaxAcc;

    // if(accRPM>maxAccRPM){ //max acceleration check
    //     motor.onOff(0);
    // } 
    if(speedRPM>tmpSetpoint){ //max speed check
        motor.onOff(0);
    } else{
        motor.onOff(1);
    }
}

int main()
{
    // virtualPWMtick.attach_us(&virtualPWM, virtualPWMPeriod);
    while(true){
        if(us_ticker_read()-t0>virtualPWMPeriod){
            virtualPWM();
            t0=us_ticker_read();
        }

        if(us_ticker_read()-t2>accelPeriod*(1000/(int)accelResolution)){
            if(tmpSetpoint> -1.0f * (baseSpeed/accelResolution)){
                tmpSetpoint-=baseSpeed/accelResolution;
            } else if(tmpSetpoint<(baseSpeed/accelResolution)){
                tmpSetpoint+=baseSpeed/accelResolution;
            } else{
                tmpSetpoint=setpoint;
            }
            t2=us_ticker_read();
        }



        if(us_ticker_read()-t1>printPeriod){
            mot.speed(0.5);
            printf("speed: %.2f speedM: %.2f goal: %.2f enc: %d delta: %d\n",speedRPM_, speedRPMM, tmpSetpoint, enc.getPulses(), encM.delta);
            t1=us_ticker_read();
        }
        // printf("%d\n",enc.getPulses());
    }
    return 0;
}