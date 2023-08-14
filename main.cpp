#include "mbed.h"
#include "Configs/Variables.h"
#include "Configs/Constants.h"
#include "Configs/ConfigurationPin.h
#include "Configs/Setup.h"
#include "../KRAI_Library/JoystickPS3/JoystickPS3.h"
#include "../KRAI_Library/Motor/Motor.h"
#include "../KRAI_Library/pidLo/pidLo.h"
#include "../KRAI_Library/SMC_KRAI/SMC_KRAI.h"
#include "../KRAI_Library/ControlMotor/ControlMotor.h"
#include "../KRAI_Library/encoderHAL/encoderHAL.h"
#include "../KRAI_Library/encoderKRAI/encoderKRAI.h"
#include "../KRAI_Library/Control4Roda/ControlAutomatic4Omni.h"
#include "../KRAI_Library/StanleyPursuit/StanleyPursuit.h"
#include "../KRAI_Library/encoderHAL/EncoderMspInitF4.h"
#include "../KRAI_Library/odom2enc/Coordinate.h"
#include "../KRAI_Library/odom2enc/odom2enc.h"
#include "../KRAI_Library/JoystickPS3/JoystickPS3.h"
#include "../KRAI_Library/PID_KRAI/PID_KRAI.h"
#include "../KRAI_Library/PIDAaronBerk/PIDAaronBerk.h"
#include "../KRAI_Library/MovingAverage/MovingAverage.h"
#include "ShooterMotor/ShooterMotor.h"

int main()
{
    

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