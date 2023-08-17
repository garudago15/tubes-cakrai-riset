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

void AngleShooter::encReset()
{
    this->encMotorAngle->reset();
}

void AngleShooter::resetMotorAngle(float pwm, bool isReset)
{
    if (isReset)
    {
        this->motorAngle->speed(pwm);
    }
}

void AngleShooter::controlAng(int targetSudut)
{
    this->angle = mapValue(this->encMotorAngle->getPulses());
    this->angle = static_cast<int>(this->movAvgAngle->movingAverage(static_cast<float>(this->angle)));

    this->outputPMWAngle = this->pidMotorAngle->createpwm(targetSudut, this->angle, 0.5);

    if (targetSudut != 80)
    {
        this->motorAngle->speed(outputPMWAngle);
    }


    

    printf("%d %d %.2f %d ", angle, targetSudut, outputPMWAngle, this->encMotorAngle->getPulses());

    printf("%f %f %f \n", getPParamLM(), getIParamLM(), getDParamLM());

}

int AngleShooter::mapValue(int encReading)
{
    int minA = 0;    // Rentang A: 0-538
    int maxA = -538;
    int minB = 80;   // Rentang B: 20-80
    int maxB = 28;

    // Pemetaan nilai dari rentang A ke rentang B
    int mappedValue = minB + static_cast<int>((encReading - minA) / static_cast<float>(maxA - minA) * (maxB - minB));

    
    return mappedValue;
}
