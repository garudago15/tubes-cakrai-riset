#ifndef CONTROL4OMNI_H
#define CONTROL4OMNI_H

#include "Control4Roda.h"
#include "../../Master_Elephant2/Configurations/Constants.h"
class Control4Omni : public Control4Roda {
    private:
        uint32_t prevFLticker = 0;
        uint32_t prevFRticker = 0;
        uint32_t prevBRticker = 0;
        uint32_t prevBLticker = 0;

    public:
        Control4Omni(Motor *FL_motor, Motor *FR_motor, Motor *BR_motor, Motor *BL_motor, encoderKRAI *encFL, encoderKRAI *encFR, encoderKRAI *encBR, encoderKRAI *encBL, 
        ControlMotor *control_FL_motor, ControlMotor *control_FR_motor, ControlMotor *control_BR_motor, ControlMotor *control_BL_motor, odom2enc *odom, 
        pidLo *vxPid, pidLo *vyPid, pidLo *wPid, StanleyPursuit *line, pidLo *pid, pidLo *pid2);

        void encoderMotorSamp();
        // virtual void updatePosition();
        virtual void baseSpeed();
        virtual void base();

        //from cakrai 15
        void basePidLo();
};

#endif 
