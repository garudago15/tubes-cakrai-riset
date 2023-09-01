#pragma once

#include "mbed.h"
#include "../../../KRAI_Library/encoderKRAI/encoderKRAI.h"
#include "../../../KRAI_Library/Motor/Motor.h"
#include "../../../KRAI_Library/pidLo/pidLo.h"
#include "../../../KRAI_Library/MovingAverage/MovingAverage.h"
#include "../Configurations/Constants.h"

class AngleShooter
{
private:
    Motor *motorAngle;
    encoderKRAI *encMotorAngle;
    pidLo *pidMotorAngle;

    // Moving Average
    MovingAverage *movAvgAngle;

    // Target
    uint32_t prevTimeNow;

    bool isDown = false;
    float outputPMWAngle;
    float omegaAngle;  // Revolutions per Minute
    
    int angleRealtime;
    int angleTarget;
    int prevPulses;

public:
    AngleShooter(Motor *motorAngle, encoderKRAI *encMotorAngle, pidLo *pidMotorAngle, MovingAverage *movAvgAngle);
    void controlAng(int targetSudut);

    void setTuning(float kp, float ki, float kd);

    int getAngleRealtime() { return this->angleRealtime; }
    int getAngleTarget() { return this->angleTarget; }

    float getPParamLM(){ return this->pidMotorAngle->getPParam(); }
    float getIParamLM(){ return this->pidMotorAngle->getIParam(); }
    float getDParamLM(){ return this->pidMotorAngle->getDParam(); }
    
    void encReset();
    void resetMotorAngle(float pwm, bool isReset);
    void setAngleTarget(int updateTarget);

    int mapValue(int encReading);
};