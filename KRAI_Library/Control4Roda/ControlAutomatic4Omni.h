#ifndef __CONTROLAUTOMATIC4OMNI_H__
#define __CONTROLAUTOMATIC4OMNI_H__

#include "Control4Omni.h"

class ControlAutomatic4Omni : public Control4Omni {
    private:
        /* Manual/Automatic settings */
        bool otomatis;

        /* Parallel park mode */
        bool parallel_park_mode;

        /* robot_to_pole_middle_mode */
        const float offset_for_lay_up = 20; // cm
        bool robot_to_pole_middle_mode;

        /* Ultrasonic */
        float ultrasonic1;
        float ultrasonic2;
        
        /* offset from pole mode */
        bool offset_from_pole_mode;
        float deltaOffset;
        /* Check if pole is on the middle of the robot*/
        bool middle;


    public:
        ControlAutomatic4Omni(Motor *FL_motor, Motor *FR_motor, Motor *BR_motor, Motor *BL_motor, encoderKRAI *encFL, encoderKRAI *encFR, encoderKRAI *encBR, encoderKRAI *encBL, 
        ControlMotor *control_FL_motor, ControlMotor *control_FR_motor, ControlMotor *control_BR_motor, ControlMotor *control_BL_motor);

        // Getter
        bool getOtomatis();
        bool getParallelParkMode();
        float getUltrasonic1();
        float getUltrasonic2();
        bool getRobotToPoleMiddleMode();
        bool getOffsetFromPoleMode();

        // Setter
        void setUltrasonic1(float ultrasonic1);
        void setUltrasonic2(float ultrasonic2);
        void setOtomatis(bool otomatis);
        void setParallelParkMode(bool parallelParkMode);
        void setRobotToPoleMiddleMode(bool robot_to_pole_middle_mode);
        void setOffsetFromPoleMode(bool offset_from_pole_mode);

        // override Control4Omni::baseSpeed()
        void baseSpeed();

        // override Control4Omni::base()
        void base();

        // override Control4Roda::updatePosition()
        void updatePosition();
};

#endif // __CONTROLAUTOMATIC4OMNI_H__
