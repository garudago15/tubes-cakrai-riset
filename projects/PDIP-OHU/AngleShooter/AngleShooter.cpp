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
    this->angleTarget = targetSudut;
    this->angleRealtime = mapValue(this->encMotorAngle->getPulses());
    this->angleRealtime = static_cast<int>(this->movAvgAngle->movingAverage(static_cast<float>(this->angleRealtime)));

    this->outputPMWAngle = this->pidMotorAngle->createpwm(targetSudut, this->angleRealtime, 0.5);

    if (targetSudut != 80)
    {
        this->motorAngle->speed(outputPMWAngle);
    }

    /* FOR DEBUGING PURPOSE (Buat tunning dan cek nyala) */
    // printf("%d %d %.2f %d ", angleRealtime, targetSudut, outputPMWAngle, this->encMotorAngle->getPulses());
    // printf("%f %f %f \n", getPParamLM(), getIParamLM(), getDParamLM());

    // Buat Tunning Lokasi
    // printf(" %d %d ", angleRealtime, targetSudut);

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

void AngleShooter::setAngleTarget(int updateTarget)
{
    this->angleTarget = updateTarget;
}
