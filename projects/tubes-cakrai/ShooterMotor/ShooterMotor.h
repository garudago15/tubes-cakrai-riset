#pragma once

#include "mbed.h"
#include "../../KRAI_Library/encoderKRAI/encoderKRAI.h"
#include "../../KRAI_Library/Motor/Motor.h"
#include "../../KRAI_Library/pidLo/pidLo.h"
#include "../../KRAI_Library/MovingAverage/MovingAverage.h"
#include "../Configurations/Constants.h"

// #define PPR_LM 3358

class  ShooterMotor
{
private:
    // Left Motor
    Motor *leftMotor;
    encoderKRAI *encLeftMotor;
    pidLo *pidLeftMotor;

    // Right Motor
    Motor *rightMotor;
    encoderKRAI *encRightMotor;
    pidLo *pidRightMotor;

    // Moving Average
    MovingAverage *movAvgLM;
    MovingAverage *movAvgRM;

    // Target
    uint32_t prevTimeNow;
    float maxRPMLM; // Revolution per Minute
    float maxRPMRM; // Revolution per Minute

    float outputPMW_LM;
    float omegaLM;  // Revolutions per Minute
    int prevPulsesLM;

    float outputPWM_RM;
    float omegaRM; // Revolution per Minute
    int prevPulsesRM;

public:
    ShooterMotor(Motor *leftMotor, Motor *rightMotor, encoderKRAI *encLeftMotor, encoderKRAI *encRightMotor,
                pidLo *pidLeftMotor, pidLo *pidRightMotor, MovingAverage *movAvgLM, MovingAverage *movAvgRM);

    void controlOmegaShooter(float setPoint);
    void setTuningLM(float kp, float ki, float kd);
    void setTuningRM(float kp, float ki, float kd);
    float getPParamLM(){ return this->pidLeftMotor->getPParam(); }
    float getIParamLM(){ return this->pidLeftMotor->getIParam(); }
    float getDParamLM(){ return this->pidLeftMotor->getDParam(); }

    float getPParamRM(){ return this->pidRightMotor->getPParam(); }
    float getIParamRM(){ return this->pidRightMotor->getIParam(); }
    float getDParamRM(){ return this->pidRightMotor->getDParam(); }
};