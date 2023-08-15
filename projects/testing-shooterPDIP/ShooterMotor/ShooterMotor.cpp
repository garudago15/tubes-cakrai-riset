#include "ShooterMotor.h"

ShooterMotor::ShooterMotor(Motor *motorFly, Motor *motorRld, Motor *motorAng
    , encoderKRAI *encFly, encoderKRAI *encRld, encoderKRAI *encAng
    , pidLo *pidFly, pidLo *pidAng, MovingAverage *movAvgFly, MovingAverage *movAvgAng
)
{
    this->motorFly = motorFly;
    this->motorRld = motorRld;
    this->motorAng = motorAng;
    this->encFly = encFly;
    this->encRld = encRld;
    this->encAng = encAng;
    this->pidFly = pidFly;
    this->pidAng = pidAng;
    this->movAvgFly = movAvgFly;
    this->movAvgAng = movAvgAng;

    this->targetRPM = 0;
    this->targetANG = 0;
    this->targetRLD = 0;
    this->currRPM = 0; // Current motorFly RPM
    this->currANG = 0; // Current motorAng Angle
    this->currRLD = 0; // Current motorRld Angle

    this->currPulseFly = 0;
    this->currPulseAng = 0;
    this->currPulseRld = 0;

    this->prevPulseFly = 0;

    this->pwmFly = 0;
    this->pwmRld = SPEED_RLD;
    this->pwmAng = 0;
    
    this->isReset = true;
    this->canShoot = true;
}

void ShooterMotor::setTargetRPM(float target){
    this->targetRPM = target;
}
void ShooterMotor::setTargetANG(float target){
    this->targetANG = target;
}
void ShooterMotor::setTargetRLD(float target){
    this->targetRLD = target;
}

float ShooterMotor::getTargetRPM(){
    return this->targetRPM;
}
float ShooterMotor::getTargetANG(){
    return this->targetANG;
}
float ShooterMotor::getTargetRLD(){
    return this->targetRLD;
}

void ShooterMotor::setTuningFly(float kp, float ki, float kd)
{
    this->pidFly->setTunings(kp, ki, kd);
}

void ShooterMotor::setTuningAng(float kp, float ki, float kd)
{
    this->pidAng->setTunings(kp, ki, kd);
}

// Steps for PID:
// 1. Get Pulses from Encoder
// 2. Convert Pulses to RPM/ANG
// 3. Apply Moving Average to RPM/ANG
// 4. Get PWM from RPM/ANG
// 5. motor.speed(pwm)
// 6. Update Previous Pulses
void ShooterMotor::controlFly()
{
    this->nowTime = us_ticker_read();
    float timeDelta = this->nowTime - this->prevTimeFly;

    this->currPulseFly = this->encFly->getPulses();
    this->currRPM = (float)(this->currPulseFly - this->prevPulseFly) / (PPR_FLY * timeDelta) * 1e6 * 60; //Pulse/(Pulse/Rotation * us) * us/s * s/m = RPM
    this->currRPM = this->movAvgFly->movingAverage(currRPM);
    this->pwmFly = this->pidFly->createpwm(this->targetRPM, this->currRPM, 1.0);
    this->motorFly->speed(pwmFly);
    this->prevPulseFly = this->currPulseFly;
    this->prevTimeFly = this->nowTime;
}
void ShooterMotor::controlAng()
{
    this->currPulseAng = this->encAng->getPulses();
    this->currANG = (float)(this->currPulseAng)/(PPR_ANG) * 360; // Pulse/(Pulse/Rotation) * degree/rotation
    this->currANG = this->movAvgAng->movingAverage(currANG);
    if (this->isReset) {
        this->pwmAng = 0.3;
    } else {
        this->pwmAng = this->pidAng->createpwm(this->targetANG, this->currANG, 1.0);
    }
    this->motorAng->speed(pwmAng);

}
void ShooterMotor::controlRld()
{
    this->currPulseRld = this->encRld->getPulses();
    this->currRLD = (float)(this->currPulseRld)/(PPR_RLD) * 360;

    if(currRPM > targetRPM){
        this->canShoot = true;
    }

    if((this->currRLD < this->targetRLD) && this->canShoot){
        this->motorRld->speed(this->pwmRld);
    } else {
        this->motorRld->forcebrake();
        this->encRld->reset();
        this->targetRLD = 0;
        this->canShoot = false;
    }
}

void ShooterMotor::reload(){
    this->setTargetRLD(180);
}
void ShooterMotor::setIsReset(bool isReset){
    this->isReset = isReset;
}
void ShooterMotor::printData(){
    printf("PulseFly:%.2f PulseAng:%.2f PulseRld:%.2f ", this->currPulseFly, this->currPulseAng, this->currPulseRld);
    printf("pwmFly:%.2f pwmAng:%.2f pwmRld:%.2f", this->pwmFly, this->pwmAng, this->pwmRld);
    printf("currRPM:%.2f currANG:%.2f, currRLD:%.2f ", this->currRPM, this->currANG, this->currRLD);
    printf("targetRPM:%.2f targetANG:%.2f, targetRLD:%2.f ", this->targetRPM, this->targetANG, this->targetRLD);
    printf("Reload ON?%d\n", (this->currRLD < this->targetRLD));
    // printf("currPulseFly:%.2f | prevPulseFly:%.2f | currRPM:%.2f | pwmFly:%.2f\n", this->currPulseFly, this->prevPulseFly, this->currRPM, this->pwmFly);
}