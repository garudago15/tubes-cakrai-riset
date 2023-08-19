#include "ShooterMotor.h"

ShooterMotor::ShooterMotor(Motor *leftMotor,encoderKRAI *encLeftMotor,pidLo *pidLeftMotor,MovingAverage *movAvgLM, MovingAverage *movAvgAccel, Motor *motorReload, encoderKRAI *encMotorReload)
{
    this->leftMotor = leftMotor;
    this->encLeftMotor = encLeftMotor;
    this->pidLeftMotor = pidLeftMotor;
    this->movAvgLM = movAvgLM;
    this->movAvgAccel = movAvgAccel;

    this->motorReload = motorReload;
    this->encMotorReload = encMotorReload;

    this->maxRPMLM = 641.0f;
    this->maxRPMRM = 855.0f;
    this->prevTimeNow = 0;
    this->accelShooter = 0;

    // this->pidLeftMotor->setInputLimits(0.0, maxRPMLM);
    // this->pidLeftMotor->setOutputLimits(0.0, 1.0);
    // this->pidRightMotor->setInputLimits(maxRPMRM, 0.0);
    // this->pidRightMotor->setOutputLimits(0.0, 1.0);

    this->outputPMW_LM = 0;
    this->prevPulsesLM = 0;
    this->omegaLM = 0;
    this->prevOmega = 0;

    this->isInReload = true;
}

void ShooterMotor::runReloader(float deltaDerajatRLD, float pwmReloader)
{
    // 360 derajat = 1 putaran
    // 104.4 pulses = 1 putaran

    // 360 derajat = 104.4 pulses    
    // x derajat = y pulses
    // maka
    // 360/x = 104.4/y

    // y[pulses] = 104.4 * x[derajat] / 360

    if (this->encMotorReload->getPulses() < PPR_RLD * deltaDerajatRLD / 360.0f)
    {
        this->motorReload->speed(pwmReloader);
        isInReload = true;
    }
    else
    {
        // this->motorReload->speed(0.0f);
        this->motorReload->forcebrake();
        this->encMotorReload->reset();
        isInReload = false;
    }
    // printf("%d\n", this->encMotorReload->getPulses());
}

void ShooterMotor::setTuningLM(float kp, float ki, float kd)
{
    this->pidLeftMotor->setTunings(kp, ki, kd);
    
    printf("Set to %f %f %f \n", kp, ki, kd);
    printf("Pakai Fungsi %f %f %f\n", getPParamLM(), getIParamLM(), getDParamLM());
}

void ShooterMotor::controlOmegaShooter(float setPoint)
{
    this->setPointFlywheel = setPoint;

    uint32_t timeNow = us_ticker_read();
    /*
    Speed Measurement
        FREQUENCY-BASED MEASUREMENT
        measured the number of encoder pulses in a fix gate time
        omega = (delta_pulses) / (PPR * timeSampling)
    */
    float beforeMovAvg;
    this->omegaLM = ((this->encLeftMotor->getPulses() - this->prevPulsesLM) / (PPR_LM * (float)(timeNow - this->prevTimeNow)/1000000.0f)) * 60; // Revolutions per Minute
    beforeMovAvg = this->omegaLM;
    this->omegaLM = this->movAvgLM->movingAverage(this->omegaLM);

    // PID dan set speed motor
    this->outputPMW_LM = this->pidLeftMotor->createpwm(setPoint, this->omegaLM, 1.0); // PWM POSITIVE => omegaLM (+)

    // PID Hanya jika diatas 0
    if (setPoint != 0)
    {
        this->leftMotor->speed(this->outputPMW_LM);
    } else {
        this->leftMotor->speed(0.0);
        this->pidLeftMotor->reset();
    }
    
    // this->leftMotor->speed(0.8f);

    // Akselerasi omega
    this->accelShooter = (this->omegaLM - this->prevOmega)/(60* (float)(timeNow - this->prevTimeNow)/1000000.0f);
    this->accelShooter = this->movAvgAccel->movingAverage(this->accelShooter);

    // printf("%f %f %f\n", this->omegaLM, this->accelShooter, (this->omegaLM - this->prevOmega)/(60* (float)(timeNow - this->prevTimeNow)/1000000.0f));
    // printf("%f %f\n", this->omegaLM, this->accelShooter);

    // Update nilai pulses
    this->prevPulsesLM = this->encLeftMotor->getPulses();
    this->prevTimeNow = us_ticker_read();
    this->prevOmega = this->omegaLM;


    // printf("%f %f\n", beforeMovAvg, this->omegaLM);

    /* BUAT TUNNING LAPANGAN */
    //printf(" %f %f ", this->omegaLM, setPoint);

    // printf("%f %f %f %f %f\n", this->omegaLM, setPoint, getPParamLM(), getIParamLM(), getDParamLM());

    // printf("%d %d\n", this->encLeftMotor->getPulses(), this->encRightMotor->getPulses());
    // printf("%f %f\n", this->omegaLM, this->omegaRM);
    // printf("%f %f %f %f %f %f\n", this->omegaLM/maxRPMLM, setPoint/maxRPMLM, this->outputPMW_LM, getPParamLM(), getIParamLM(), getDParamLM());
    // printf("%f %f %f %f %f\n", this->omegaRM, setPoint, getPParamRM(), getIParamRM(), getDParamRM());
    // printf("%f %f %f %f %f %f\n", this->omegaRM/maxRPMRM, setPoint/maxRPMRM, this->outputPWM_RM, getPParamRM(), getIParamRM(), getDParamRM());
    // printf("%f %f %f\n", this->omegaRM/maxRPM, setPoint/maxRPM, this->outputPWM_RM);
}


float ShooterMotor::getCalcRPM(float x, int targetAngle)
{
    if (targetAngle == 55)
    {
        return (-104.4*x*x + 827.3*x + 1391);
    } else if (targetAngle == 65)
    {
        return (245.9*x*x + 655.5*x + 1259);
    } else if (targetAngle == 70)
    {
        return (-745.1*x*x + 1021*x + 2070);
    } else {
        return 0.0;
    }
}