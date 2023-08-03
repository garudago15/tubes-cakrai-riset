#ifndef _THROWER_H_
#define _THROWER_H_

#include "mbed.h"
#include "../../KRAI_Library/Motor/Motor.h"
#include "../../KRAI_Library/encoderKRAI/encoderKRAI.h"
#include "../../KRAI_Library/servoKRAI/servoKRAI.h"
#include "../../KRAI_Library/pneumaticKRAI/pneumaticKRAI.h"
#include "../../KRAI_Library/pidlo/pidLo.h"
#include "../../KRAI_Library/INA219/INA219.h"
#include "../Configurations/State.h"
#include "../Configurations/Constants.h"
#include "../../KRAI_Library/PID_KRAI/PID_KRAI.h"
#include "../../KRAI_Library/PIDAaronBerk/PIDAaronBerk.h"
#include "ThrowingMode.h"

class Shooter
{
private:
    Motor *aimMotor; // Linear_motor untuk aiming
    INA219 *encoderAimMotor;

    // Roller untuk friction wheel shooter
    Motor *rollerTop;
    encoderKRAI *encoderRollerTop;
    PIDAaronBerk *pidRollerTop;

    Motor *rollerBottom;
    encoderKRAI *encoderRollerBottom;
    PIDAaronBerk *pidRollerBottom;

    // Status Object
    char state = STATE_SHOOTER_NOT_READY;
    bool magazineReady = false;

    // Aiming
    float angle;
    float angleTarget;
    float aimPWM;

    // Roller => Speed dalam Rad/s
    int rollerTopPulse;
    float rollerTopSpeed;
    float rollerTopPWM;
    float rollerSpeedStartTime = 0;
    float rollerStartTime = 0;

    int rollerBottomPulse;
    float rollerBottomSpeed;
    float rollerBottomPWM;

    ThrowingMode currentThrowingMode;

    bool rollerActive = false;

    // Ring Pusher
    pneumaticKRAI* ringPusher;

    // Helper
    float getPotentioAngle();

public:
    // Ctor
    Shooter(Motor *aimMotor, Motor *rollerTop, Motor *rollerBottom, pneumaticKRAI *ringPusher, INA219 *encoderAimMotor,
        encoderKRAI *encoderRollerTop, encoderKRAI *encoderRollerBottom, PIDAaronBerk *pidRollerTop, PIDAaronBerk *pidRollerBottom);

    Shooter(Motor *aimMotor, Motor *rollerTop, Motor *rollerBottom, pneumaticKRAI *ringPusher, INA219 *encoderAimMotor,
        encoderKRAI *encoderRollerTop, encoderKRAI *encoderRollerBottom, PIDAaronBerk *pidRollerTop, PIDAaronBerk *pidRollerBottom, ThrowingMode ThrowingMode);

    // Main Logic
    void run();

    // Roller Mechanism
    void rollerSampling();
    void activateRoller();
    void deactivateRoller();

    // Aim Mechanism
    void aimSampling();
    void aiming();

    // Shoot Mechanism
    void shoot();

    /*Setter*/
    void setState(char state);
    void setAngleTarget(float targetAngle);
    void setSpeedTargetRollerTop(float targetSpeed);
    void setSpeedTargetRollerBottom(float targetSpeed);
    void setAimPWM(float pwm);
    void setmagazineReady(bool magazineReady);
    void setRollerActive(bool rollerActive);
    void setSpeedTarget(ThrowingMode throwingMode);
    void setSpeedTarget(float topTargetSpeed, float bottomTargetSpeed);
    void setMode(ThrowingMode throwingMode);



    /*Getter*/
    char getState();
    float getAngle();
    float getAngleTarget();
    float getRollerTopSpeed();
    float getRollerTopSpeedTarget();
    float getRollerBottomSpeed();
    float getRollerBottomSpeedTarget();
    ThrowingMode getThrowingMode();
    float getAimPWM();
    bool isRollerActive();
};

#endif
