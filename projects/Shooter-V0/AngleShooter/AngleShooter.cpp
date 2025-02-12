#include "AngleShooter.h"

AngleShooter::AngleShooter(Motor *motorAngle, encoderKRAI *encMotorAngle, pidLo *pidMotorAngle, MovingAverage *movAvgAngle)
{
    this->motorAngle = motorAngle;
    this->encMotorAngle = encMotorAngle;
    this->pidMotorAngle = pidMotorAngle;
    this->movAvgAngle = movAvgAngle;


    this->prevTimeNow = 0;

    this->outputPMWAngle = 0.0f;
    this->omegaAngle = 0.0f;  // Revolutions per Minute
    this->prevPulses = 0;
}

void AngleShooter::setTuning(float kp, float ki, float kd)
{
    this->pidMotorAngle->setTunings(kp, ki, kd);
}

void AngleShooter::controlAng(float targetSudut)
{
    this->outputPMWAngle = this->pidMotorAngle->createpwm(targetSudut, this->encMotorAngle->getPulses(), 0.8);
    this->motorAngle->speed(outputPMWAngle);
}