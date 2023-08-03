#include "Shooter.h"

// Roller Top 104 PPR


// ctor
Shooter::Shooter(Motor *aimMotor, Motor *rollerTop, Motor *rollerBottom, pneumaticKRAI *ringPusher, INA219 *encoderAimMotor,
                 encoderKRAI *encoderRollerTop, encoderKRAI *encoderRollerBottom, PIDAaronBerk *pidRollerTop, PIDAaronBerk *pidRollerBottom) : currentThrowingMode(ThrowingMode(0, 0))
{
    this->aimMotor = aimMotor;
    this->rollerTop = rollerTop;
    this->rollerBottom = rollerBottom;
    this->ringPusher = ringPusher;
    this->encoderAimMotor = encoderAimMotor;
    this->encoderRollerTop = encoderRollerTop;
    this->encoderRollerBottom = encoderRollerBottom;
    this->pidRollerTop = pidRollerTop;

    this->pidRollerTop->setInputLimits(-1, 1);
    this->pidRollerTop->setOutputLimits(-1, 1);

    this->pidRollerBottom = pidRollerBottom;

    this->aimPWM = 1.0;
}

Shooter::Shooter(Motor *aimMotor, Motor *rollerTop, Motor *rollerBottom, pneumaticKRAI *ringPusher, INA219 *encoderAimMotor,
                 encoderKRAI *encoderRollerTop, encoderKRAI *encoderRollerBottom, PIDAaronBerk *pidRollerTop, PIDAaronBerk *pidRollerBottom, ThrowingMode throwingMode) : currentThrowingMode(throwingMode)
{
    this->aimMotor = aimMotor;
    this->rollerTop = rollerTop;
    this->rollerBottom = rollerBottom;
    this->ringPusher = ringPusher;
    this->encoderAimMotor = encoderAimMotor;
    this->encoderRollerTop = encoderRollerTop;
    this->encoderRollerBottom = encoderRollerBottom;
    this->pidRollerTop = pidRollerTop;
    this->pidRollerBottom = pidRollerBottom;

    this->aimPWM = 1.0;
}

// Getter
char Shooter::getState()
{
    return this->state;
}
float Shooter::getAngle()
{
    return this->angle;
}
float Shooter::getAngleTarget()
{
    return this->angleTarget;
}
float Shooter::getRollerTopSpeed()
{
    return this->rollerTopSpeed;
}
float Shooter::getRollerTopSpeedTarget()
{
    return this->currentThrowingMode.getTopTargetSpeed();
}
float Shooter::getRollerBottomSpeed()
{
    return this->rollerBottomSpeed;
}
float Shooter::getRollerBottomSpeedTarget()
{
    return this->currentThrowingMode.getBottomTargetSpeed();
}
ThrowingMode Shooter::getThrowingMode()
{
    return this->currentThrowingMode;
}

float Shooter::getAimPWM()
{
    return this->aimPWM;
}

bool Shooter::isRollerActive()
{
    return this->rollerActive;
}

// SETTER
void Shooter::setState(char state)
{
    this->state = state;
}
void Shooter::setAngleTarget(float targetAngle)
{
    this->angleTarget = targetAngle;
}
void Shooter::setSpeedTargetRollerTop(float targetSpeed)
{
    this->currentThrowingMode.setSpeed(targetSpeed, this->currentThrowingMode.getBottomTargetSpeed());
}
void Shooter::setSpeedTargetRollerBottom(float targetSpeed)
{
    this->currentThrowingMode.setSpeed(this->currentThrowingMode.getTopTargetSpeed(), targetSpeed);
}

void Shooter::setAimPWM(float pwm)
{
    this->aimPWM = pwm;
}

void Shooter::setmagazineReady(bool magazineReady)
{
    this->magazineReady = magazineReady;
}

void Shooter::setRollerActive(bool rollerActive)
{
    this->rollerActive = rollerActive;
}

void Shooter::setSpeedTarget(ThrowingMode throwingMode)
{
    this->currentThrowingMode.setSpeed(throwingMode);
}

void Shooter::setSpeedTarget(float topTargetSpeed, float bottomTargetSpeed)
{
    this->currentThrowingMode.setSpeed(topTargetSpeed, bottomTargetSpeed);
}

void Shooter::setMode(ThrowingMode throwingMode)
{
    setSpeedTarget(throwingMode);
}

// AIM MECHANISM
void Shooter::aimSampling()
{
    // Nanti bakalan ada library untuk potentiometer slider =>
    this->angle = this->getPotentioAngle();
}
void Shooter::aiming()
{
    // Angkat aim
    if (this->angleTarget > (this->angle+MARGIN_ERROR_AIM))
    {
        this->state = STATE_SHOOTER_AIM_UP;
        // printf("Belum sampai, Naikkan aim...  Target: %f  Current: %f\n", angleTarget, angle);
    }
    else if (this->angleTarget < this->angle- MARGIN_ERROR_AIM)
    {
        this->state = STATE_SHOOTER_AIM_DOWN;
        // printf("Belum sampai, Turunkan aim...  Target: %f  Current: %f\n", angleTarget, angle);
    }
    else
    {
        this->aimMotor->speed(0);

        if (rollerActive && magazineReady)
        {
            this->state = STATE_SHOOTER_READY;
        }
        else
        {
            this->state = STATE_SHOOTER_NOT_READY;
        }
        // printf("Sampai euy meureen ceunah! Target: %f  Current: %f\n", angleTarget, angle);
    }
}

float Shooter::getPotentioAngle()
{
    // Ambil output dari encoderLinearMotor convert dalam angle
    return encoderAimMotor->getVoltage() * RANGE_AIM_ANGLE / RANGE_VOLTAGE_ADC;
}

// ROLLER MECHANISM
void Shooter::rollerSampling()
{

    if (us_ticker_read() - this->rollerStartTime > SAMPLING_ROLLER)
    {
        int now = us_ticker_read();

        this->rollerTopPulse = this->encoderRollerTop->getPulses();
        this->rollerBottomPulse = this->encoderRollerBottom->getPulses();

        // printf("PULSE rollerTop: %d\t rollerBottom: %d\n", this->rollerTopPulse, this->rollerBottomPulse);

        // this->rollerTopPulse = this->encoderRollerTop->delta;
        // this->rollerTopSpeed =8 * 60 * 1000000 / PPR /  (this->rollerTopPulse);
        this->rollerTopSpeed = (float)(this->rollerTopPulse) * 60 * 1000000 / (PPR * float(now - this->rollerStartTime)); // 104 PPR

        // this->rollerBottomPulse = this->encoderRollerBottom->delta;
        // this->rollerBottomSpeed = 8 *60  * 1000000 / PPR / (this->rollerBottomPulse);
        this->rollerBottomSpeed = (float)(this->rollerBottomPulse) * 60  * 1000000 / (PPR * float(now - this->rollerStartTime));

        // printf("Speed top: %f\t", this->rollerTopSpeed);
        // printf("Speed bottom: %f\n", this->rollerBottomSpeed);

        encoderRollerTop->reset();
        encoderRollerBottom->reset();

        this->rollerStartTime = us_ticker_read();
    }
}

void Shooter::activateRoller()
{
    this->rollerTopPWM = this->pidRollerTop->createpwm(getRollerTopSpeedTarget(), this->rollerTopSpeed, MAX_PWM_ROLLER_TOP);
    this->rollerBottomPWM = this->pidRollerBottom->createpwm(getRollerBottomSpeedTarget(), this->rollerBottomSpeed, MAX_PWM_ROLLER_BOTTOM);
    // printf("PWM rollerTop: %f\t rollerBottom: %f\n", this->rollerTopPWM, this->rollerBottomPWM);
    // this->rollerTopPWM = 0;
    // this->rollerBottomPWM = 0;

    this->rollerTop->speed(this->rollerTopPWM);
    this->rollerBottom->speed(this->rollerBottomPWM);
    this->rollerActive = true;
}

void Shooter::deactivateRoller()
{
    this->rollerTop->speed(0);
    this->rollerBottom->speed(0);
    this->rollerActive = false;
}

// Shoot Mechanism
void Shooter::shoot()
{
    if (this->state == STATE_SHOOTER_NOT_READY && magazineReady && rollerActive)
    {
        this->state = STATE_SHOOTER_READY;
    }
    if (this->state == STATE_SHOOTER_READY)
    {
        setState(STATE_SHOOTER_SHOOT);
    }
}

// MAIN LOGIC
void Shooter::run()
{
    // aiming();
    // aimSampling();
    rollerSampling();
    switch (this->state)
    {
    case STATE_SHOOTER_AIM_UP:
        this->aimMotor->speed(this->aimPWM);
        break;
    case STATE_SHOOTER_AIM_DOWN:
        this->aimMotor->speed(-this->aimPWM);
        break;
    case STATE_SHOOTER_READY:
        this->aimMotor->speed(0);
        break;
    case STATE_SHOOTER_SHOOT:
        this->ringPusher->Extend();
        this->state = STATE_SHOOTER_READY;
        break;
    }

    // printf("Speed top: %f\t", this->rollerTopSpeed);

    // printf("Speed bottom: %f\n", this->rollerBottomSpeed);
}
