/* mbed Microcontroller Library
 * Copyright (c) 2023 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mbed.h"
#include "../../KRAI_Library/Motor/Motor.h"
// #include "../../KRAI_Library/encoderKRAI/encoderKRAI.h"
#include "../../KRAI_Library/pidLo/pidLo.h" // Include the pidLo library
// #include "../../KRAI_Library/Pinout/F407VET6_2023.h"

#include "../../libs/Configs/Constants.h"
#include "../../libs/Configs/ConfigurationPin.h"
// #include "../../KRAI_Library/Pinout/F446RE_MASTER_2022.h"
#include "../../KRAI_Library/Pinout/F407VET6_2023.h"
#include "../../libs/MovingAverage/MovingAverage.h"
#include "../../libs/noPWMMotor/noPWMMotor.h"
#include "../../libs/encoderMotorSpeed/encoderMotorSpeed.h"

#define resolution 50

// PIN Encoder
#define CHA ENCODER_SHOOTER_CHA
#define CHB ENCODER_SHOOTER_CHB

// PIN Motor
#define PWM MOTOR_SHOOTER_PWM
#define FOR MOTOR_SHOOTER_FOR
#define REV MOTOR_SHOOTER_REV

/* INITIALIZE SERIAL USING PRINTF */
static BufferedSerial serial_port(PA_9, PA_10, 115200);
FileHandle *mbed::mbed_override_console(int fd){
    return &serial_port;
}
#define PPR 105
Motor mot(PWM, FOR, REV);
// noPWMMotor motor(PWM, FOR, REV);
// encoderKRAI enc(CHA, CHB, ENC_BASE_MOTOR_PPR / 2, X2_ENCODING);
encoderMotorSpeed encM(CHA, CHB, PPR / 2, X2_ENCODING);
MovingAverage Avg(20);

#define pidPeriod 5000 // 10 ms, movavg 20, 
uint32_t tPID=0;
float Kp=0.001f;
float Ki=0.0f;
float Kd=0.0f;
pidLo pid(Kp, Ki, Kd, (float)pidPeriod/1000000.0f, 1, 0, 1000, 1000);

float lastRPM=0;
uint32_t t0=0, t1=0, t2=0, tAVG=0;
#define virtualPWMPeriod 500 //in us
#define printPeriod 100000

#define absoluteMaxAcc 1.0f
#define TIMEOUT 1000000
#define baseSpeed 600.0f
uint32_t accelPeriod = 1000; //in ms
float lastSetpoint=0.0f, setpoint=0.0f; //120.0f
float tmpSetpoint=0.0f;
float toRPMMultiplier= 60.0f * 1000000.0f / (float)PPR; //dibagi virtual PWM period
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
bool stable=true;

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
    realSpeedRPM = (float)(encM.getPulses() - tmpPulse) * toRPMMultiplier / (float)(us_ticker_read() - tAVG);
    tmpPulse=encM.getPulses();
    tAVG=us_ticker_read();
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
        mot.speed(0.0f);
        // motor.forcebrake();
    } else{
        mot.speed(1.0f);
    }
}

int main()
{
    placeHolder_setup();
    // virtualPWMtick.attach_us(&virtualPWM, virtualPWMPeriod);
    while(true){
        if (serial_port.readable())
        {
            lastSetpoint=setpoint;
            scanf("%f %f %f %f", &setpoint, &Kp, &Ki, &Kd);
            pid.setTunings(Kp, Ki, Kd);
            if(setpoint!=lastSetpoint){
                stable=false;
                output=1.0f;
                // virtualPWMtick.attach_us(&virtualPWM, virtualPWMPeriod);
            }
            
        }
        // if(us_ticker_read()-t0>virtualPWMPeriod){
        //     virtualPWM();
        //     t0=us_ticker_read();
        // }

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
                if(stable==false){
                    // virtualPWMtick.detach();
                    stable=true;
                    pid.reset();
                }
            }
            t2=us_ticker_read();
        }

        if(stable==true){
            if(us_ticker_read()-tPID>pidPeriod){
                speedRPM=getAvgSpeed();
                output=pid.createpwm(tmpSetpoint, speedRPM, 1.0f);
                // mot.speed(output);
                mot.speed(0.3f);
                tPID=us_ticker_read();
            }
        }

        



        if(us_ticker_read()-t1>printPeriod){
            // mot.speed(0.3);
            // printf("speed: %.2f goal: %.2f enc: %d\n",speedRPM_, tmpSetpoint, enc.getPulses());
            printf("speed: %.2f goal: %.2f pwm: %f enc: %d delta: %d Kp:%.5f Ki:%.5f Kd:%.5f\n", speedRPM, tmpSetpoint, output, encM.getPulses(), encM.delta, Kp, Ki, Kd);
            // printf("speed: %.2f speedM: %.2f goal: %.2f enc: %d delta: %d\n",speedRPM_, speedRPMM, tmpSetpoint, enc.getPulses(), encM.delta);
            t1=us_ticker_read();
        }
        // printf("%d\n",enc.getPulses());
    }
    return 0;
}