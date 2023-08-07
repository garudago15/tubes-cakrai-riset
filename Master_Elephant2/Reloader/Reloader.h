#ifndef _RELOADER_H_
#define _RELOADER_H_

#include "mbed.h"
#include "../../KRAI_Library/Motor/Motor.h"
#include "../../KRAI_Library/encoderKRAI/encoderKRAI.h"
#include "../../KRAI_Library/servoKRAI/servoKRAI.h"
#include "../../KRAI_Library/pidLo/pidLo.h"
#include "../Configurations/State.h"
#include "../Configurations/Constants.h"

// Negatif counter-clockwise jika dilihat dari bawah servo (arah luar robot)

class Reloader
{
private:

    servoKRAI *servo;
    DigitalIn *limit;
    // Status
    char state = STATE_RELOADER_STANDBY_POSITION;
    float targetAngle;
    float shakeDegree;
    float standbyPositionAngle = POSITION_STANDBY_ANGLE;
    float positionDownAngle = POSITION_DOWN_ANGLE;
    bool limitReached;

    // Internal variable
    uint32_t reloader_last_toggle = us_ticker_read();
    uint32_t reloader_last_moved = us_ticker_read();

public:
    Reloader(servoKRAI *servo, DigitalIn *limit);

    void run();
    void recalibrateAngle();
    void toggleReloader();
    void reset();
    void shakeUp();
    void shakeDown();

    /*Setter*/
    void setState(char state);
    void setStandbyPositionAngle(float angle);
    void setPositionDownAngle(float angle);
    void setShakeDegree(float degree);

    /*Getter*/
    char getState();
    int getTargetAngle();
    float getStandbyPositionAngle();
    float getPositionDownAngle();
    float getShakeDegree();
    bool isLimitReached();
};

#endif