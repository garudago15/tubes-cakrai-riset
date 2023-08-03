#ifndef ODOM2ENC_H
#define ODOM2ENC_H

#include "mbed.h"
#include "../encoderKRAI/encoderKRAI.h"
#include "../CMPS12_KRAI/CMPS12_KRAI.h"
#include "Coordinate.h"

class odom2enc{
    private:
        encoderKRAI *encX;
        encoderKRAI *encY;

        int xCurrentPulse, yCurrentPulse;
    public:
        odom2enc(encoderKRAI *encX, encoderKRAI *encY);
        Coordinate position;
        void resetOdom(void);
        void resetEncoder(void);
        void updatePosition(void);
        void setPosition(float x_input, float y_input);
};

#endif