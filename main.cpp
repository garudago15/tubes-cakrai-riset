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
    const int MAX_MSG = 64;
    static char message[MAX_MSG];
    static float kpKiKd[3];
    int constPID_pos = 0;
    static unsigned int message_pos = 0;
    char inByte;

    while (1)
    {
        while (serial_port.readable())
        {
            serial_port.read(&inByte, 1);

            if ((inByte != '\r') && message_pos < MAX_MSG - 1)
            {
                message[message_pos] = inByte;
                message_pos++;
            }
            else
            {
                message[message_pos] = '\0';
                message_pos = 0;
                char *token = strtok(message, " ");

                while (token != NULL)
                {
                    kpKiKd[constPID_pos] = atof(token);
                    // printf("%s\n", token);
                    token = strtok(NULL, " ");
                    constPID_pos++;
                    if (constPID_pos == 3)
                    {
                        constPID_pos = 0;
                    }
                }

                // Change left pid left motor tuning
                controlShooterMotor.setTuningRM(kpKiKd[0], kpKiKd[1], kpKiKd[2]);
                // printf("kp = %f || ki = %f || kd = %f\n", pidLeftMotor.getPParam(), pidLeftMotor.getIParam(), pidLeftMotor.getDParam());
                // printf("kp = %f || ki = %f || kd = %f\n", kpKiKd[0], kpKiKd[1], kpKiKd[2]);
            }
        }

        if (us_ticker_read() - timeLast > samplingPID)
        {
            // printf("%d\n", rotEncAdjSpeed.getPulses());
            // controlShooterMotor.controlOmegaShooter((float)rotEncAdjSpeed.getPulses()); 
            controlShooterMotor.controlOmegaShooter(trySetPoint); 
            timeLast = us_ticker_read();
        }
    }

    return 0;
}