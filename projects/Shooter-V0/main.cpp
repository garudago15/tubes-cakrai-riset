#include "mbed.h"
#include "../../KRAI_Library/encoderKRAI/encoderKRAI.h"
#include "../../KRAI_Library/Motor/Motor.h"
#include "../../KRAI_Library/PIDAaronBerk/PIDAaronBerk.h"
#include "../../KRAI_Library/MovingAverage/MovingAverage.h"
#include "Configurations/Constants.h"
#include "Configurations/Setup.h"
#include "ShooterMotor/ShooterMotor.h"
#include "../../KRAI_Library/JoystickPS3/JoystickPS3.h"

// Inisialisasi Pengendali Motor Shooter
ShooterMotor controlShooterMotor(&leftMotor, &encLeftMotor, &pidLeftMotor, &movAvgLM, &movAvgAccel, &motorReload, &encMotorReload);
JoystickPS3 ps3(UART_STICK_TX, UART_STICK_RX);

int main()
{
    ps3.setup();  

    float trySetPoint = 0.0f; // SetPoint PID (RPM)
    /* Debugin Purpose */
    const int MAX_MSG = 64;
    static char message[MAX_MSG];
    static float kpKiKd[4];
    int constPID_pos = 0;
    static unsigned int message_pos = 0;
    char inByte;

    bool startReload = false;

    while (1)
    {
        ps3.olah_data();
        ps3.baca_data();

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
                    if (constPID_pos == 4)
                    {
                        constPID_pos = 0;
                    }
                }

                // Change left pid left motor tuning
                controlShooterMotor.setTuningLM(kpKiKd[0], kpKiKd[1], kpKiKd[2]);
                trySetPoint = kpKiKd[3];
                // printf("kp = %f || ki = %f || kd = %f\n", pidLeftMotor.getPParam(), pidLeftMotor.getIParam(), pidLeftMotor.getDParam());
                // printf("kp = %f || ki = %f || kd = %f\n", kpKiKd[0], kpKiKd[1], kpKiKd[2]);
            }
        }
        

        // ------------------------------------------- UNTUK RELOAD ---------------------------------------------------------------------

        if (ps3.getKotak() && (controlShooterMotor.getAccelShooter() > -5.0f && controlShooterMotor.getAccelShooter() < 5.0f) &&
        (controlShooterMotor.getOmegaShooter() - trySetPoint > -10.0f && controlShooterMotor.getOmegaShooter() - trySetPoint < 10.0f))
        {
            startReload = true;
        }

        if (startReload)
        {
            if (controlShooterMotor.getReloaderStatus() == false)
            {
                startReload = false;
                controlShooterMotor.setReloaderStatus(true);
            }
            else
            {
                controlShooterMotor.runReloader(180.0, 0.7);
            }
        }

        // ---------------------------------------------------------------------------------------------------------------------------

        if (us_ticker_read() - timeLast > samplingPID)
        
            controlShooterMotor.controlOmegaShooter(trySetPoint); 
            timeLast = us_ticker_read();
        }
    }

    return 0;
}