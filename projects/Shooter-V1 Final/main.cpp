#include "mbed.h"
#include "../../KRAI_Library/encoderKRAI/encoderKRAI.h"
#include "../../KRAI_Library/Motor/Motor.h"
#include "../../KRAI_Library/PIDAaronBerk/PIDAaronBerk.h"
#include "../../KRAI_Library/MovingAverage/MovingAverage.h"
#include "Configurations/Constants.h"
#include "Configurations/Setup.h"
#include "ShooterMotor/ShooterMotor.h"
#include "AngleShooter/AngleShooter.h"
#include "../../KRAI_Library/JoystickPS3/JoystickPS3.h"

// Inisialisasi Pengendali Motor Shooter
ShooterMotor controlShooterMotor(&leftMotor, &encLeftMotor, &pidLeftMotor, &movAvgLM, &movAvgAccel, &motorReload, &encMotorReload);
AngleShooter controlAngShooter(&motorAngle, &encMotorAngle, &pidMotorAngle, &movAvgANG);
JoystickPS3 ps3(UART_STICK_TX, UART_STICK_RX);

// ------------------------------------------------------------------

bool interruptState = false;
void riseMotor(){
    if(LimitSwitch.read()){
        interruptState = true;
        controlAngShooter.encReset();
        RESET_ANG_MOTOR = false;
    }
}
void fallMotor(){
    if(!LimitSwitch.read()){
        interruptState = false;
    }
}

// --------------------------------------------------------------------


int main()
{
    ps3.setup();

    LimitSwitch.rise(&riseMotor);
    LimitSwitch.fall(&fallMotor);

    float trySetPoint = 0.0f; // SetPoint PID (RPM)
    int angleSetPoint = 80;
    bool startReload = false;

    /* Debugin Purpose */
    float Kp, Ki, Kd;
    // -------------------

    while (1)
    {
        ps3.olah_data();
        ps3.baca_data();

        // ---------------------------- TUNNING FLYWHEEL ------------------------------
        // if (serial_port.readable())
        // {
        //     scanf("%f %f %f", &Kp, &Ki, &Kd);
        //     controlShooterMotor.setTuningLM(Kp, Ki, Kd);
        //     printf(" OUTSIDE %f %f %f \n", controlShooterMotor.getDParamLM(), controlShooterMotor.getIParamLM(), controlShooterMotor.getDParamLM());
        // }
        // printf(" %f %f %f \n", controlShooterMotor.getDParamLM(), controlShooterMotor.getIParamLM(), controlShooterMotor.getDParamLM());
        // ----------------------------------------------------------------------------


        // ----------------------------- TUNNING SUDUT -------------------------------
        if (serial_port.readable())
        {
            scanf("%f %f %f", &Kp, &Ki, &Kd);
            controlAngShooter.setTuning(Kp, Ki, Kd);
            printf(" OUTSIDE %f %f %f \n", controlAngShooter.getDParamLM(), controlShooterMotor.getIParamLM(), controlShooterMotor.getDParamLM());
        }
        // printf(" %f %f %f \n", controlAngShooter.getDParamLM(), controlShooterMotor.getIParamLM(), controlShooterMotor.getDParamLM());
        // ---------------------------------------------------------------------------
        

        // ------------------------- RELOAD MECHANISM -------------------------
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
        // ------------------------------------------------------------------------------





        // ------------------------- CONTROL MOTOR SHOOTER -------------------------
        if (ps3.getLingkaran() && ps3.getButtonDown())
        {
            trySetPoint = 0;
        } else if (ps3.getLingkaran() && ps3.getButtonLeft())
        {
            trySetPoint = 2900;
        } else if (ps3.getLingkaran() && ps3.getButtonUp())
        {
            trySetPoint = 3200;
        }

        if (us_ticker_read() - timeLast > samplingPID)
        { 
            controlShooterMotor.controlOmegaShooter(trySetPoint);
        }
        // ------------------------------------------------------------------------------


        // ------------------------- CONTROL ANGLE SHOOTER -------------------------
        
        if (ps3.getLY() < -120)
        {
            angleSetPoint = 80;
        } else if (ps3.getLY() > 120)
        {
            angleSetPoint = 60;
        } 

        // printf("%d %d\n", ps3.getLY(), angleSetPoint);       // Buat debug bener ga ni tombol

        // Reset Angle Purpose
        if (ps3.getLY() < -120 && ps3.getRY() < -120)
        {
            RESET_ANG_MOTOR = true;
            angleSetPoint = 80;                                 // Posisi Paling Bawah
        }
        controlAngShooter.resetMotorAngle(0.35, RESET_ANG_MOTOR);

        // Sampling for PID Purpose
        if (us_ticker_read() - timeLastForAngleShooter > samplingPID)
        {
            controlAngShooter.controlAng(angleSetPoint);
            timeLastForAngleShooter = us_ticker_read();
        }
        // ------------------------------------------------------------------------------

    }

    return 0;
}