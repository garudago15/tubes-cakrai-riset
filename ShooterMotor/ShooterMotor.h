#pragma once

#include "mbed.h"
#include "../../KRAI_Library/encoderKRAI/encoderKRAI.h"
#include "../../KRAI_Library/Motor/Motor.h"
#include "../../KRAI_Library/PIDAaronBerk/PIDAaronBerk.h"
#include "../../KRAI_Library/MovingAverage/MovingAverage.h"
#include "../Configurations/Constants.h"

// #define PPR_LM 3358

class  ShooterMotor
{
private:
    // Left Motor
    Motor *leftMotor;
    encoderKRAI *encLeftMotor;
    PIDAaronBerk *pidLeftMotor;

    // Right Motor
    Motor *rightMotor;
    encoderKRAI *encRightMotor;
    PIDAaronBerk *pidRightMotor;

    // Moving Average
    MovingAverage *movAvgLM;
    MovingAverage *movAvgRM;

    // Target
    uint32_t prevTimeNow;
    float maxRPM; // Revolution per Minute

    float outputPMW_LM;
    float omegaLM;  // Revolutions per Minute
    int prevPulsesLM;

    float outputPWM_RM;
    float omegaRM; // Revolution per Minute
    int prevPulsesRM;

public:
    ShooterMotor(Motor *leftMotor, Motor *rightMotor, encoderKRAI *encLeftMotor, encoderKRAI *encRightMotor,
                PIDAaronBerk *pidLeftMotor, PIDAaronBerk *pidRightMotor, MovingAverage *movAvgLM, MovingAverage *movAvgRM);

    void controlOmegaShooter(float setPoint);
    void setTuningLM(float kp, float ki, float kd);
    void setTuningRM(float kp, float ki, float kd);
};