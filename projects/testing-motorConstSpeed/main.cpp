/* mbed Microcontroller Library
 * Copyright (c) 2023 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mbed.h"
#include "../../KRAI_Library/Motor/Motor.h"
// #include "../../KRAI_Library/encoderKRAI/encoderKRAI.h"
// #include "../../KRAI_Library/pidLo/pidLo.h" // Include the pidLo library
// #include "../../KRAI_Library/Pinout/F407VET6_2023.h"

#include "../../libs/Configs/Constants.h"
// #include "../../libs/Configs/ConfigurationPin.h"
// #include "../../KRAI_Library/Pinout/F446RE_MASTER_2022.h"
#include "../../KRAI_Library/Pinout/F407VET6_2023.h"
#include "../../libs/MovingAverage/MovingAverage.h"
#include "../../libs/noPWMMotor/noPWMMotor.h"
#include "../../libs/encoderMotorSpeed/encoderMotorSpeed.h"

#define resolution 50

// PIN Encoder
#define CHA F407VET6_ENCODER_2_2_A
#define CHB F407VET6_ENCODER_2_2_B

// PIN Motor
#define PWM F407VET6_PWM_MOTOR_8
#define FOR F407VET6_FOR_MOTOR_8
#define REV F407VET6_REV_MOTOR_8

/* INITIALIZE SERIAL USING PRINTF */
static BufferedSerial serial_port(PA_9, PA_10, 115200);
FileHandle *mbed::mbed_override_console(int fd){
    return &serial_port;
}

// Motor mot(PWM, FOR, REV);
noPWMMotor motor(PWM, FOR, REV);
// encoderKRAI enc(CHA, CHB, ENC_BASE_MOTOR_PPR / 2, X2_ENCODING);
encoderMotorSpeed encM(CHA, CHB, ENC_BASE_MOTOR_PPR / 2, X2_ENCODING);
MovingAverage Avg(20);

float lastRPM=0;
uint32_t t0=0, t1=0, t2=0;
#define virtualPWMPeriod 500 //in us
#define printPeriod 100000
#define PPR 105
#define absoluteMaxAcc 1.0f
#define TIMEOUT 1000000
#define baseSpeed 120.0f
uint32_t accelPeriod = 1000; //in ms
float setpoint=0.0f; //120.0f
float tmpSetpoint=0.0f;
float toRPMMultiplier= 60.0f * 1000000.0f / (float)virtualPWMPeriod / (float)PPR;
float toRPMMultiplierM = 60.0f * 1000000.0f / (float)PPR;
float toAccRPMMultiplier = 1000000.0f / (float)virtualPWMPeriod;

float speedRPM, realSpeedRPM, accRPM, maxAccRPM;
float speedRPM_, speedRPMM;
int32_t tmpPulse = encM.getPulses();
float tmpSpeedRPM=0.0f;

float  output;
float motor_default_speed=0.5;

float accelResolution=100.0f;
int j=0;
float AVG=0.0f;

float placeHolder[resolution];
void placeHolder_setup(){
    for(int i=0; i<resolution; i++){
        placeHolder[i]=0.0f;
    }
}
float fast_movingAvg(float data){
    AVG+=(data - placeHolder[j])/resolution;
    placeHolder[j]=data;
    j++;
    if(j>=resolution){
        j=0;
    }
    return AVG;
}


float getAvgSpeed(){
    realSpeedRPM = (float)(encM.getPulses() - tmpPulse) * toRPMMultiplier;
    tmpPulse=encM.getPulses();
    return fast_movingAvg(realSpeedRPM);
}

// float getAvgSpeedM(){
//     if(us_ticker_read() - encM.prevt > TIMEOUT){
//         return 0.0f;
//     } else{
//         return toRPMMultiplierM/(float)encM.delta;
//     }
// }

Ticker virtualPWMtick;
void virtualPWM(){
    speedRPM_ = getAvgSpeed();
    // speedRPMM = getAvgSpeedM();

    speedRPM = speedRPM_; //change this
    // accRPM = (speedRPM - tmpSpeedRPM) * toAccRPMMultiplier;

    // //taking tmp values
    
    // tmpSpeedRPM=speedRPM;

    // maxAccRPM=absoluteMaxAcc;

    // if(accRPM>maxAccRPM){ //max acceleration check
    //     motor.onOff(0);
    // } 
    if(speedRPM>=tmpSetpoint){ //max speed check
        motor.onOff(0);
        // motor.forcebrake();
    } else{
        motor.onOff(1);
    }
}

int main()
{
    placeHolder_setup();
    // virtualPWMtick.attach_us(&virtualPWM, virtualPWMPeriod);
    while(true){
        if (serial_port.readable())
        {

            scanf("%d %f", &resolution, &setpoint);
        }
        if(us_ticker_read()-t0>virtualPWMPeriod){
            virtualPWM();
            t0=us_ticker_read();
        }

        // if(us_ticker_read()-t2>accelPeriod*(1000/(int)accelResolution)){
        //     if(tmpSetpoint> -1.0f * (baseSpeed/accelResolution)){
        //         tmpSetpoint-=baseSpeed/accelResolution;
        //     } else if(tmpSetpoint<(baseSpeed/accelResolution)){
        //         tmpSetpoint+=baseSpeed/accelResolution;
        //     } else{
        //         tmpSetpoint=setpoint;
        //     }
        //     t2=us_ticker_read();
        // }

        if(us_ticker_read()-t2>accelPeriod*(1000/(int)accelResolution)){
            if(tmpSetpoint> setpoint + baseSpeed/accelResolution){
                tmpSetpoint-=baseSpeed/accelResolution;
            } else if(tmpSetpoint<setpoint - baseSpeed/accelResolution){
                tmpSetpoint+=baseSpeed/accelResolution;
            } else{
                tmpSetpoint=setpoint;
            }
            t2=us_ticker_read();
        }



        if(us_ticker_read()-t1>printPeriod){
            // mot.speed(0.3);
            // printf("speed: %.2f goal: %.2f enc: %d\n",speedRPM_, tmpSetpoint, enc.getPulses());
            printf("speed: %.2f goal: %.2f enc: %d delta: %d\n", speedRPM, tmpSetpoint, encM.getPulses(), encM.delta);
            // printf("speed: %.2f speedM: %.2f goal: %.2f enc: %d delta: %d\n",speedRPM_, speedRPMM, tmpSetpoint, enc.getPulses(), encM.delta);
            t1=us_ticker_read();
        }
        // printf("%d\n",enc.getPulses());
    }
    return 0;
}