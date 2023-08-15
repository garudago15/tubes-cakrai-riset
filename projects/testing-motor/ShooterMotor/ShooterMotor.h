#pragma once

#include "mbed.h"
#include "../../KRAI_Library/encoderKRAI/encoderKRAI.h"
#include "../../KRAI_Library/Motor/Motor.h"
#include "../../KRAI_Library/pidLo/pidLo.h"
#include "../../KRAI_Library/MovingAverage/MovingAverage.h"
#include "../Configurations/Constants.h"

// #define PPR_LM 3358

class ShooterMotor
{
private:
    // Left Motor
    Motor *leftMotor;
    encoderKRAI *encLeftMotor;
    pidLo *pidLeftMotor;

    Motor *motorReload;
    encoderKRAI *encMotorReload;

    // Moving Average
    MovingAverage *movAvgLM;
    MovingAverage *movAvgAccel;

    // Target
    uint32_t prevTimeNow;
    float maxRPMLM; // Revolution per Minute
    float maxRPMRM; // Revolution per Minute

    float outputPMW_LM;
    float omegaLM;  // Revolutions per Minute
    float prevOmega;
    int prevPulsesLM;

    int prevPulsesReloader;
    bool isInReload;
    float accelShooter;

public:
    ShooterMotor(Motor *leftMotor,encoderKRAI *encLeftMotor,pidLo *pidLeftMotor,MovingAverage *movAvgLM, MovingAverage *movAvgAccel, Motor *motorReload, encoderKRAI *encMotorReload);
    // ShooterMotor(Motor *leftMotor, Motor *rightMotor, encoderKRAI *encLeftMotor, encoderKRAI *encRightMotor,
    //             pidLo *pidLeftMotor, pidLo *pidRightMotor, MovingAverage *movAvgLM, MovingAverage *movAvgRM);

    void controlOmegaShooter(float setPoint);
    void setTuningLM(float kp, float ki, float kd);
    float getOmegaShooter(){ return this->omegaLM; }
    float getAccelShooter(){ return this->accelShooter; }
    float getPParamLM(){ return this->pidLeftMotor->getPParam(); }
    float getIParamLM(){ return this->pidLeftMotor->getIParam(); }
    float getDParamLM(){ return this->pidLeftMotor->getDParam(); }

    void runReloader(float deltaDerajatRLD, float pwmReloader);
    void setReloaderStatus(bool flag) {this->isInReload = flag; }
    bool getReloaderStatus(){ return this->isInReload; }
};