#include "ShooterMotor.h"

ShooterMotor::ShooterMotor(Motor *leftMotor, Motor *rightMotor, encoderKRAI *encLeftMotor, encoderKRAI *encRightMotor,
                PIDAaronBerk *pidLeftMotor, PIDAaronBerk *pidRightMotor, MovingAverage *movAvgLM, MovingAverage *movAvgRM)
{
    this->leftMotor = leftMotor;
    this->rightMotor = rightMotor;
    this->encLeftMotor = encLeftMotor;
    this->encRightMotor = encRightMotor;
    this->pidLeftMotor = pidLeftMotor;
    this->pidRightMotor = pidRightMotor;
    this->movAvgLM = movAvgLM;
    this->movAvgRM = movAvgRM;
    this->maxRPM = 500.0f;
    this->prevTimeNow = 0;

    this->pidLeftMotor->setInputLimits(-maxRPM, maxRPM);
    this->pidLeftMotor->setOutputLimits(-1.0, 1.0);
    this->pidRightMotor->setInputLimits(0.0, maxRPM);
    this->pidRightMotor->setOutputLimits(-1.0, 1.0);

    this->outputPMW_LM = 0;
    this->outputPWM_RM = 0;

    this->prevPulsesLM = 0;
    this->prevPulsesRM = 0;

    this->omegaLM = 0;
    this->omegaRM = 0;
}

void ShooterMotor::setTuningLM(float kp, float ki, float kd)
{
    this->pidLeftMotor->setTunings(kp, ki, kd);
}

void ShooterMotor::setTuningRM(float kp, float ki, float kd)
{
    this->pidRightMotor->setTunings(kp, ki, kd);
}

void ShooterMotor::controlOmegaShooter(float setPoint)
{
    // PWM POSITIF MENGHASILKAN MOTOR CLOCK WISE

    uint32_t timeNow = us_ticker_read();
    /*
    Speed Measurement
        FREQUENCY-BASED MEASUREMENT
        measured the number of encoder pulses in a fix gate time
        omega = (delta_pulses) / (PPR * timeSampling)
    */
   // Arah CCW => PWM (-)
    this->omegaLM = ((this->encLeftMotor->getPulses() - this->prevPulsesLM) / (PPR_LM * (float)(timeNow - this->prevTimeNow)/1000000.0f)) * 60; // Revolutions per Minute
    this->omegaLM = this->movAvgLM->movingAverage(omegaLM);

    // Ingin Arah CW => PWM (+)
    this->omegaRM = ((this->encRightMotor->getPulses() - this->prevPulsesRM) / (PPR_RM * (float)(timeNow - this->prevTimeNow)/1000000.0f)) * 60; // Revolutions per Minute
    this->omegaRM = this->movAvgRM->movingAverage(omegaRM);

    // PID dan set speed motor
    this->outputPMW_LM = this->pidLeftMotor->advancepwm(-setPoint, this->omegaLM, 1.0);
    this->leftMotor->speed(this->outputPMW_LM);
    // this->leftMotor->speed(-0.25f);


    this->outputPWM_RM = this->pidRightMotor->createpwm(setPoint, this->omegaRM, 1.0);
    // this->rightMotor->speed(this->outputPWM_RM);
    // this->rightMotor->speed(0.25f);

    // Constant Speed
    // this->leftMotor->speed(0.45f);

    // Update nilai pulses
    this->prevPulsesLM = this->encLeftMotor->getPulses();
    this->prevPulsesRM = this->encRightMotor->getPulses();
    this->prevTimeNow = us_ticker_read();

    // Debug: print data omega
    // printf("%f %f\n", testWithoutMovAvg, this->omegaLM);
    // printf("%f\n", omegaLM);
    // printf("%d %d\n", this->encLeftMotor->getPulses(), this->encRightMotor->getPulses());
    // printf("%f %f\n", this->omegaLM, this->omegaRM);
    printf("%f %f %f\n", this->omegaLM/maxRPM, -setPoint/maxRPM, this->outputPMW_LM);
    // printf("%f %f %f\n", this->omegaRM/maxRPM, setPoint/maxRPM, this->outputPWM_RM);
}