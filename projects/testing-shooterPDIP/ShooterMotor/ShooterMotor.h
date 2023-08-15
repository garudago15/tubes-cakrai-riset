#pragma once

#include "mbed.h"
#include "../../../KRAI_Library/encoderKRAI/encoderKRAI.h"
#include "../../../KRAI_Library/Motor/Motor.h"
#include "../../../KRAI_Library/pidLo/pidLo.h"
#include "../../../KRAI_Library/MovingAverage/MovingAverage.h"
#include "../Configurations/Constants.h"

// #define PPR_LM 3358

class ShooterMotor{
    private:
        // --- Motor Flywheel ---
        Motor *motorFly; 
        encoderKRAI *encFly;
        pidLo *pidFly;
        MovingAverage *movAvgFly;

        // --- Motor Reloader ---
        Motor *motorRld;
        encoderKRAI *encRld;

        // --- Motor Angle ---
        Motor *motorAng;
        encoderKRAI *encAng;
        pidLo *pidAng;
        MovingAverage *movAvgAng;

        // --- Target Variable ---
        float targetRPM;
        float targetANG;
        float targetRLD;

        // --- Other Variable ---
        // -- Current Pulses --
        float currPulseFly; // Current encFly Pulses
        float currPulseAng; // Current encAng Pulses
        float currPulseRld; // Current encRld Pulses
        // -- Previous Pulses for RPM -- 
        float prevPulseFly; // Previous encFly Pulses
        // -- Current RPM/ANG --
        float currRPM; // Current motorFly RPM
        float currANG; // Current motorAng Angle
        float currRLD; // Current motorRld Angle
        // --- TS Variable ---
        float nowTime; // Time Now
        float prevTimeFly; // Previous Time Sampled

        // --- PWM Variable ---
        float pwmFly;
        float pwmRld;
        float pwmAng;

        // --- RESET ANGLE ---
        bool isReset;
        bool canShoot;


    public:
        ShooterMotor(Motor *motorFly, Motor *motorRld, Motor *motorAng
            , encoderKRAI *encFly, encoderKRAI *encRld, encoderKRAI *encAng
            , pidLo *pidFly, pidLo *pidAng, MovingAverage *movAvgFly, MovingAverage *movAvgAng
        );

        // Set Target
        void setTargetRPM(float target);
        void setTargetANG(float target);
        void setTargetRLD(float target);
        // Get Target
        float getTargetRPM();
        float getTargetANG();
        float getTargetRLD();

        // Set Tuning Motor Fly/Rld/Ang
        void setTuningFly(float kp, float ki, float kd);
        void setTuningAng(float kp, float ki, float kd);

        // Control Motor (Baca Encoder + PWM/PID + MOTOR.speed)
        void controlFly();
        void controlAng();
        void controlRld();

        // Reloader
        void reload();
        // Reset Angle
        void setIsReset(bool isReset);

        // Other
        void printData();
};