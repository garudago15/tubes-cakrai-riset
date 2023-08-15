#include "mbed.h"
#include "Configs/Variables.h"
#include "Configs/Constants.h"
#include "Configs/ConfigurationPin.h
#include "Configs/Setup.h"
#include "KRAI_Library/JoystickPS3/JoystickPS3.h"
#include "KRAI_Library/Motor/Motor.h"
#include "KRAI_Library/pidLo/pidLo.h"
#include "KRAI_Library/SMC_KRAI/SMC_KRAI.h"
#include "KRAI_Library/ControlMotor/ControlMotor.h"
#include "KRAI_Library/encoderHAL/encoderHAL.h"
#include "KRAI_Library/encoderKRAI/encoderKRAI.h"
#include "KRAI_Library/Control4Roda/ControlAutomatic4Omni.h"
#include "KRAI_Library/StanleyPursuit/StanleyPursuit.h"
#include "KRAI_Library/encoderHAL/EncoderMspInitF4.h"
#include "KRAI_Library/odom2enc/Coordinate.h"
#include "KRAI_Library/odom2enc/odom2enc.h"
#include "KRAI_Library/JoystickPS3/JoystickPS3.h"
#include "KRAI_Library/PID_KRAI/PID_KRAI.h"
#include "KRAI_Library/PIDAaronBerk/PIDAaronBerk.h"
#include "KRAI_Library/MovingAverage/MovingAverage.h"
#include "ShooterMotor/ShooterMotor.h"

int main()
{
    printf("STICK START\n");

    /* SET UP JOYSTICK */
    ps3.idle();   
    ps3.setup();
    omni.encoderMotorSamp(); //baca data awal encoder omniWheel

    while (1)
    {   
        /* OMNI */
        // Pengolahan dan Update data PS3
        ps3.olah_data();
        ps3.baca_data();
        
        // utk reset
        if (ps3.getStart())
        {
            NVIC_SystemReset();
            printf("start");
        }

        //default
        vx_cmd = 0;
        vy_cmd = 0;
        w_cmd = 0;

        //braking system
        if(ps3.getKotak()){ //tombol utk nembak
            omni.forceBrakeSync(); //hard brake sebelum nembak agar robot tidak gerak saat nembak
        } else{
            //moving
            if (ps3.getButtonRight()) { //gerak ke kanan
                vx_cmd += TRANSLATION_BASE_SPEED;
            } 
            if (ps3.getButtonLeft()) { //gerak ke kiri
                vx_cmd -= TRANSLATION_BASE_SPEED;
            } 

            if (ps3.getButtonUp()) { //gerak ke atas
                vy_cmd += TRANSLATION_BASE_SPEED;
            } 
            if (ps3.getButtonDown()) { //gerak ke bawah
                vy_cmd -= TRANSLATION_BASE_SPEED;
            }

            if (ps3.getL1()) { //spin berlawanan arah jarum jam
                w_cmd += ROTATION_BASE_SPEED;
            } 
            if (ps3.getR1()) { //spin berlawanan arah jarm jam
                w_cmd -= ROTATION_BASE_SPEED;
            }

            if (vx_cmd == 0.0 && vy_cmd == 0.0 && w_cmd == 0.0) {
                // Change controllers parameters if the base about to stop wholly (soft braking)
                smcMotorBL.setKp(BASE_BL_SMC_KP_BRAKE);
                smcMotorBR.setKp(BASE_BR_SMC_KP_BRAKE);
                smcMotorFL.setKp(BASE_FL_SMC_KP_BRAKE);
                smcMotorFR.setKp(BASE_FR_SMC_KP_BRAKE);

                smcMotorBL.setKsigma(SMC_BL_KSIGMA_BRAKE);
                smcMotorBR.setKsigma(SMC_BR_KSIGMA_BRAKE);
                smcMotorFL.setKsigma(SMC_FL_KSIGMA_BRAKE);
                smcMotorFR.setKsigma(SMC_FR_KSIGMA_BRAKE);

                pidMotorBL.setKp(BASE_BL_KP_BRAKE);
                pidMotorBR.setKp(BASE_BR_KP_BRAKE);
                pidMotorFL.setKp(BASE_FL_KP_BRAKE);
                pidMotorFR.setKp(BASE_FR_KP_BRAKE);    
            } else {
                // Change back controller parameters if base is moving

                if (ps3.getR2()) {
                    // Boost mode, also change controllers parameter
                    vx_cmd *= TRANSLATION_BOOST_MULTIPLIER;
                    vy_cmd *= TRANSLATION_BOOST_MULTIPLIER;
                    w_cmd *= ROTATION_BOOST_MULTIPLIER;

                    smcMotorBL.setKp(BASE_BL_SMC_KP_BOOST);
                    smcMotorBR.setKp(BASE_BR_SMC_KP_BOOST);
                    smcMotorFL.setKp(BASE_FL_SMC_KP_BOOST);
                    smcMotorFR.setKp(BASE_FR_SMC_KP_BOOST);
                } else { //normal mode
                    smcMotorBL.setKp(BASE_BL_SMC_KP);
                    smcMotorBR.setKp(BASE_BR_SMC_KP);
                    smcMotorFL.setKp(BASE_FL_SMC_KP);
                    smcMotorFR.setKp(BASE_FR_SMC_KP);
                }

                smcMotorBL.setKsigma(SMC_BL_KSIGMA);
                smcMotorBR.setKsigma(SMC_BR_KSIGMA);
                smcMotorFL.setKsigma(SMC_FL_KSIGMA);
                smcMotorFR.setKsigma(SMC_FR_KSIGMA);

                pidMotorBL.setKp(BASE_BL_KP);
                pidMotorBR.setKp(BASE_BR_KP);
                pidMotorFL.setKp(BASE_FL_KP);
                pidMotorFR.setKp(BASE_FR_KP);
            }

            //stick sampling
            if (us_ticker_read() - samplingStick > SAMP_STICK_US)
            {
                // stick.stickState(&vx_cmd, &vy_cmd, &w_cmd);

                omni.set_vx_cmd(vx_cmd);
                omni.set_vy_cmd(vy_cmd);
                omni.set_w_cmd(w_cmd);

                // printf("PS3: vx_cmd: %f, vy_cmd: %f, w_cmd: %f\n", vx_cmd, vy_cmd, w_cmd);

                samplingStick = us_ticker_read();
            }

            /********************** ENCODER **********************/

            if (us_ticker_read() - samplingEncoder > SAMP_BASE_MOTOR_ENCODER_US)
            {
                omni.encoderMotorSamp();

                //printing curr speed
                // float junk1;
                
                // omni.getVars(&junk1,&junk1,&junk1,&v_FL_curr,&v_FR_curr,&v_BR_curr,&v_BL_curr,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1);
                printf("vFL: %.2f vFR: %.2f vBL: %.2f vBR: %.2f\n", omni.get_v_FL_curr(), omni.get_v_FR_curr(), omni.get_v_BL_curr(), omni.get_v_BR_curr());
                // printf("FL: %d FR: %d BL:%d BR: %d\n", encoderBaseFL.getPulses(), encoderBaseFR.getPulses(), encoderBaseBL.getPulses(), encoderBaseBR.getPulses());

                samplingEncoder = us_ticker_read();
            }

            /********************** PID & ODOM **********************/

            if (us_ticker_read() - samplingPID > SAMP_PID_BASE_MOTOR_US) //update PID berdasarkan target dan current speed tiap motor
            {
                omni.pidMotorSamp();

                samplingPID = us_ticker_read();
            }

            if (us_ticker_read() - samplingOdom > SAMP_BASE_ODOMETRY_US) //update odometri
            {
                omni.baseSpeed();

                samplingOdom = us_ticker_read();
            }

            /********************** MOTOR **********************/

            if (us_ticker_read() - samplingMotor > SAMP_BASE_MOTOR_US) //update motor
            {
                samplingMotor = us_ticker_read();

                omni.motorSamp();
            }

            /********************** IK **********************/

            if (us_ticker_read() - samplingIK > SAMP_IK_US) //update target speed tiap motor berdasarkan vx, vy, dan w
            {
                omni.base();

                samplingIK = us_ticker_read();
            }
            /********************** PRINTS **********************/

            // // Jalanin semua roda di base
            // baseMotorBL.speed(0.4);
            // baseMotorBR.speed(0.4);
            // baseMotorFL.speed(0.4);
            // baseMotorFR.speed(0.4);

            // PRINTF("FR : %d, FL : %d, BR : %d, BL : %d\n", encoderBaseFR.getPulses(), encoderBaseFL.getPulses(), encoderBaseBR.getPulses(), encoderBaseBL.getPulses());

        }
        
        /* SHOOTER */
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