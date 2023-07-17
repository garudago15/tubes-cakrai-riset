#include "mbed.h"
#include "../../libs/Configs/Constants.h"
#include "../../libs/Configs/ConfigurationPin.h"
// #include "../../libs/Control4Roda/Control4Omni.h"
#include "../../KRAI_Library/Motor/Motor.h"
#include "../../KRAI_Library/pidLo/pidLo.h"
#include "../../KRAI_Library/SMC_KRAI/SMC_KRAI.h"
#include "../../KRAI_Library/ControlMotor/ControlMotor.h"
#include "../../KRAI_Library/encoderHAL/encoderHAL.h"
#include "../../KRAI_Library/encoderKRAI/encoderKRAI.h"
#include "../../KRAI_Library/Control4Roda/ControlAutomatic4Omni.h"
#include "../../KRAI_Library/StanleyPursuit/StanleyPursuit.h"
#include "../../KRAI_Library/encoderHAL/EncoderMspInitF4.h"
#include "../../KRAI_Library/odom2enc/Coordinate.h"
#include "../../KRAI_Library/odom2enc/odom2enc.h"
// #include "../../KRAI_Library/odom3enc/odom3enc.h"
#include "../../KRAI_Library/JoystickPS3/JoystickPS3.h"
#include "../../KRAI_Library/PID_KRAI/PID_KRAI.h"

/* INITIALIZE SERIAL USING PRINTF */
static BufferedSerial serial_port(PA_9, PA_10, 115200);
FileHandle *mbed::mbed_override_console(int fd){
    return &serial_port;
}

/* INITIALIZE PS3 JOYSTICK */
JoystickPS3 ps3(PS3_TX, PS3_RX);

/* Base Motor */
Motor baseMotorFL(BASE_MOTOR_FL_PWM, BASE_MOTOR_FL_FOR, BASE_MOTOR_FL_REV);
Motor baseMotorFR(BASE_MOTOR_FR_PWM, BASE_MOTOR_FR_FOR, BASE_MOTOR_FR_REV);
Motor baseMotorBL(BASE_MOTOR_BL_PWM, BASE_MOTOR_BL_FOR, BASE_MOTOR_BL_REV);
Motor baseMotorBR(BASE_MOTOR_BR_PWM, BASE_MOTOR_BR_FOR, BASE_MOTOR_BR_REV);

/* PID Vx, Vy, W */
pidLo vxPid(0, 0, 0, SAMP_IK_US, 10, 1, 1000, 100);
pidLo vyPid(0, 0, 0, SAMP_IK_US, 10, 1, 1000, 100);
pidLo wPid(0.025, 0.0025, 0, SAMP_IK_US, 10, 1, 1000, 100);

PID pidMotorFL(BASE_FL_KP, BASE_FL_KI, BASE_FL_KD, BASE_FL_TS_MS, 0.5, 6);
PID pidMotorFR(BASE_FR_KP, BASE_FR_KI, BASE_FR_KD, BASE_FR_TS_MS, 0.5, 6);
PID pidMotorBR(BASE_BR_KP, BASE_BR_KI, BASE_BR_KD, BASE_BR_TS_MS, 0.5, 6);
PID pidMotorBL(BASE_BL_KP, BASE_BL_KI, BASE_BL_KD, BASE_BL_TS_MS, 0.5, 8);

/* SMC untuk motor */
SMC smcMotorFL(BASE_FL_SMC_KP, SMC_FL_KSIGMA, SMC_EPSILON, SMC_BETA, (float)SAMP_PID_BASE_MOTOR_US / MS_TO_US, SMC::KECEPATAN);
SMC smcMotorFR(BASE_FR_SMC_KP, SMC_FR_KSIGMA, SMC_EPSILON, SMC_BETA, (float)SAMP_PID_BASE_MOTOR_US / MS_TO_US, SMC::KECEPATAN);
SMC smcMotorBR(BASE_BR_SMC_KP, SMC_BR_KSIGMA, SMC_EPSILON, SMC_BETA, (float)SAMP_PID_BASE_MOTOR_US / MS_TO_US, SMC::KECEPATAN);
SMC smcMotorBL(BASE_BL_SMC_KP, SMC_BL_KSIGMA, SMC_EPSILON, SMC_BETA, (float)SAMP_PID_BASE_MOTOR_US / MS_TO_US, SMC::KECEPATAN);

/* Control Motor */
ControlMotor controlMotorFL(&pidMotorFL, &smcMotorFL, BASE_MOTOR_V_LIM, BASE_FL_KP, BASE_FL_KP_MAX, BASE_FL_KD, BASE_FL_KD_MAX, BASE_FL_SMC_KP, BASE_FL_SMC_KP_MAX);
ControlMotor controlMotorFR(&pidMotorFR, &smcMotorFR, BASE_MOTOR_V_LIM, BASE_FR_KP, BASE_FR_KP_MAX, BASE_FR_KD, BASE_FR_KD_MAX, BASE_FR_SMC_KP, BASE_FR_SMC_KP_MAX);
ControlMotor controlMotorBR(&pidMotorBR, &smcMotorBR, BASE_MOTOR_V_LIM, BASE_BR_KP, BASE_BR_KP_MAX, BASE_BR_KD, BASE_BR_KD_MAX, BASE_BR_SMC_KP, BASE_BR_SMC_KP_MAX);
ControlMotor controlMotorBL(&pidMotorBL, &smcMotorBL, BASE_MOTOR_V_LIM, BASE_BL_KP, BASE_BL_KP_MAX, BASE_BL_KD, BASE_BL_KD_MAX, BASE_BL_SMC_KP, BASE_BL_SMC_KP_MAX);

/* Base Motor Encoder */
encoderKRAI encoderBaseFL(ENCODER_BASE_MOTOR_FL_CHA, ENCODER_BASE_MOTOR_FL_CHB, ENC_BASE_MOTOR_PPR / 2, X2_ENCODING);
encoderKRAI encoderBaseFR(ENCODER_BASE_MOTOR_FR_CHA, ENCODER_BASE_MOTOR_FR_CHB, ENC_BASE_MOTOR_PPR / 2, X2_ENCODING);
encoderKRAI encoderBaseBR(ENCODER_BASE_MOTOR_BR_CHA, ENCODER_BASE_MOTOR_BR_CHB, ENC_BASE_MOTOR_PPR / 2, X2_ENCODING);
encoderKRAI encoderBaseBL(ENCODER_BASE_MOTOR_BL_CHA, ENCODER_BASE_MOTOR_BL_CHB, ENC_BASE_MOTOR_PPR / 2, X2_ENCODING);

/* Odometry Encoder */
encoderKRAI encL(ENCODER_L_CHA, ENCODER_L_CHB, 1000, X4_ENCODING);
encoderKRAI encAux(ENCODER_AUX_CHA, ENCODER_AUX_CHB, 1000, X4_ENCODING);

/* Odometry */
odom2enc odom(&encAux, &encL);

/* Trajectory Following */
StanleyPursuit line;
pidLo pid(0.08, 0.05, 0.5, 0.5, 0.5, 0, 1000, 100);
pidLo pid2(0.03, 0.005, 0.2, 0.5, 0.5, 0, 1000, 100);

/* ControlKRAI */
ControlAutomatic4Omni omni(&baseMotorFL, &baseMotorFR, &baseMotorBR, &baseMotorBL, &encoderBaseFL, &encoderBaseFR, &encoderBaseBR, &encoderBaseBL, &controlMotorFL, &controlMotorFR, &controlMotorBR, &controlMotorBL, &odom, &vxPid, &vyPid, &wPid, &line, &pid, &pid2);

//omni variable
float vx_cmd = 0, vy_cmd = 0, w_cmd = 0;


/* INITIALIZE TIMERS*/
/* Time Sampling for Base */
uint32_t samplingStick = 0; 
uint32_t samplingPID = 0;
uint32_t samplingOdom = 0;
// uint32_t samplingStickRead = 0;
uint32_t samplingMotor = 0;
// uint32_t samplingUpdPos = 0;
// uint32_t samplingEncoder = 0;
uint32_t samplingIK = 0;
// uint32_t samplingPrint = 0;
// uint32_t samplingOtomatis = 0;
// uint32_t samplingOtomatisESP = 0;
// uint32_t samplingParallelPark = 0;
// uint32_t samplingAutoAim = 0;

int main(){
    printf("STICK START\n");

    /* SET UP JOYSTICK */
    stick.idle();   
    stick.setup();
    omni.encoderMotorSamp(); //baca data awal encoder omniWheel
    while (true){
        // Pengolahan dan Update data PS3
        ps3.olah_data();
        ps3.baca_data();
        
        // utk reset
        if (ps3.getStart())
        {
            NVIC_SystemReset();
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
            if (stick.getButtonRight()) { //gerak ke kanan
                vx_cmd = TRANSLATION_BASE_SPEED;
            } else if (stick.getButtonLeft()) { //gerak ke kiri
                vx_cmd = -1.0f * TRANSLATION_BASE_SPEED;
            } else{ //default
                vx_cmd = 0.0f;
            }

            if (stick.getButtonUp()) { //gerak ke atas
                vy_cmd = TRANSLATION_BASE_SPEED;
            } else if (stick.getButtonDown()) { //gerak ke bawah
                vy_cmd = -1.0f * TRANSLATION_BASE_SPEED;
            } else{ //default
                vy_cmd = 0.0f;
            }

            if (stick.getL1()) { //spin berlawanan arah jarum jam
                w_cmd = ROTATION_BASE_SPEED;
            } else if (stick.getR1()) { //spin berlawanan arah jarm jam
                w_cmd = -1.0f * ROTATION_BASE_SPEED;
            } else{ //default
                w_cmd = 0.0f;
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

                if (stick.getR2()) {
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

                // PRINTF("PS3: vx_cmd: %f, vy_cmd: %f, w_cmd: %f\n", vx_cmd, vy_cmd, w_cmd);

                samplingStick = us_ticker_read();
            }

            /********************** PID & ODOM **********************/

            if (us_ticker_read() - samplingPID > SAMP_PID_BASE_MOTOR_US) //update PID berdasarkan target dan current speed tiap motor
            {
                omni.pidMotorSamp();

                samplingPID = us_ticker_read();
            }

            if (us_ticker_read() - samplingOdom > SAMP_BASE_ODOMETRY_US) //update odometri
            {
                controlElephant.baseSpeed();

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

        // old sampling method
        // now = us_ticker_read();
        // if(now - timer1 > TIME_SAMPLING){
        //     //omni sampling
        //     omni.encoderMotorSamp(); //baca encoder (current speed) tiap motor
        //     omni.baseSpeed(); //update odometri
        //     omni.base(); //update target speed tiap motor berdasarkan vx, vy, dan w
        //     omni.pidMotorSamp(); //update PID berdasarkan target dan current speed tiap motor
        //     omni.motorSamp(); //update motor

        //     timer1 = now;
        // }
    }
}