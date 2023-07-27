#include "mbed.h"
#include "../../KRAI_Library/encoderKRAI/encoderKRAI.h"
#include "../../KRAI_Library/Motor/Motor.h"
#include "../../KRAI_Library/PIDAaronBerk/PIDAaronBerk.h"
#include "../../KRAI_Library/MovingAverage/MovingAverage.h"
#include "Configurations/Constants.h"
#include "Configurations/Setup.h"
#include "ShooterMotor/ShooterMotor.h"

// Inisialisasi Pengendali Motor Shooter
ShooterMotor controlShooterMotor(&leftMotor, &rightMotor, &encLeftMotor, &encRightMotor, &pidLeftMotor, &pidRightMotor, &movAvgLM, &movAvgRM);
int main()
{
    float trySetPoint = 250.0f; // SetPoint PID (RPM)
    /* Debugin Purpose */
    // int flagLeftTrue = 1;
    // float kp;
    // float ki;
    // float kd;

    while (1)
    {
        if (serial_port.readable())
        {
            // Parse the received string to extract KP, KI, KD values
            scanf("%f %f %f", &kp_LM, &ki_LM, &kd_LM);
            // scanf("%d %f %f %f", &flagLeftTrue, &kp, &ki, &kd);
            printf("Updated PID values: Kp=%.2f, Ki=%.2f, Kd=%.2f\n", kp_LM, ki_LM, kd_LM);
            controlShooterMotor.setTuningLM(kp_LM, ki_LM, kd_LM);

            // // Set the updated PID values
            // if (flagLeftTrue)
            // {
            //     controlShooterMotor.setTuningLM(kp, ki, kd);
            // }
            // else
            // {
            //     controlShooterMotor.setTuningRM(kp, ki, kd);
            // }
        }

        if (us_ticker_read() - timeLast > samplingPID)
        {
            controlShooterMotor.controlOmegaShooter(trySetPoint); 
            timeLast = us_ticker_read();
        }
    }

    return 0;
}