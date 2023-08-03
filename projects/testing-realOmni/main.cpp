#include "mbed.h"
#include "../../libs/Configs/Constants.h"
#include "../../libs/Configs/ConfigurationPin.h"
#include "../../libs/MovingAverage/MovingAverage.h"
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
JoystickPS3 ps3(UART_STICK_TX, UART_STICK_RX);

/*initialize moving average*/
MovingAverage movAvg(10);

/* Base Motor */
Motor baseMotorFL(BASE_MOTOR_FL_PWM, BASE_MOTOR_FL_FOR, BASE_MOTOR_FL_REV);
Motor baseMotorFR(BASE_MOTOR_FR_PWM, BASE_MOTOR_FR_FOR, BASE_MOTOR_FR_REV);
Motor baseMotorBL(BASE_MOTOR_BL_PWM, BASE_MOTOR_BL_FOR, BASE_MOTOR_BL_REV);
Motor baseMotorBR(BASE_MOTOR_BR_PWM, BASE_MOTOR_BR_FOR, BASE_MOTOR_BR_REV);

/* PID Vx, Vy, W */
pidLo vxPid(0, 0, 0, SAMP_IK_US, 10, 1, 1000, 100);
pidLo vyPid(0, 0, 0, SAMP_IK_US, 10, 1, 1000, 100);
pidLo wPid(0.025, 0.0025, 0, SAMP_IK_US, 10, 1, 1000, 100);

pidLo pidLoMotorFL(2.0f, 0.008f, 0.0f, BASE_FL_TS_MS, 1.0, 0, 1000, 1000);
pidLo pidLoMotorFR(2.0f, 0.008f, 0.0f, BASE_FL_TS_MS, 1.0, 0, 1000, 1000);
pidLo pidLoMotorBL(2.0f, 0.008f, 0.0f, BASE_FL_TS_MS, 1.0, 0, 1000, 1000);
pidLo pidLoMotorBR(2.0f, 0.008f, 0.0f, BASE_FL_TS_MS, 1.0, 0, 1000, 1000);

/* SMC untuk motor */
SMC smcMotorFL(BASE_FL_SMC_KP, SMC_FL_KSIGMA, SMC_EPSILON, SMC_BETA, (float)SAMP_PID_BASE_MOTOR_US / MS_TO_US, SMC::KECEPATAN);
SMC smcMotorFR(BASE_FR_SMC_KP, SMC_FR_KSIGMA, SMC_EPSILON, SMC_BETA, (float)SAMP_PID_BASE_MOTOR_US / MS_TO_US, SMC::KECEPATAN);
SMC smcMotorBR(BASE_BR_SMC_KP, SMC_BR_KSIGMA, SMC_EPSILON, SMC_BETA, (float)SAMP_PID_BASE_MOTOR_US / MS_TO_US, SMC::KECEPATAN);
SMC smcMotorBL(BASE_BL_SMC_KP, SMC_BL_KSIGMA, SMC_EPSILON, SMC_BETA, (float)SAMP_PID_BASE_MOTOR_US / MS_TO_US, SMC::KECEPATAN);

/* Control Motor */
ControlMotor controlMotorFL(&pidLoMotorFL, &smcMotorFL, BASE_MOTOR_V_LIM, BASE_FL_KP, BASE_FL_KP_MAX, BASE_FL_KD, BASE_FL_KD_MAX, BASE_FL_SMC_KP, BASE_FL_SMC_KP_MAX);
ControlMotor controlMotorFR(&pidLoMotorFR, &smcMotorFR, BASE_MOTOR_V_LIM, BASE_FR_KP, BASE_FR_KP_MAX, BASE_FR_KD, BASE_FR_KD_MAX, BASE_FR_SMC_KP, BASE_FR_SMC_KP_MAX);
ControlMotor controlMotorBR(&pidLoMotorBR, &smcMotorBR, BASE_MOTOR_V_LIM, BASE_BR_KP, BASE_BR_KP_MAX, BASE_BR_KD, BASE_BR_KD_MAX, BASE_BR_SMC_KP, BASE_BR_SMC_KP_MAX);
ControlMotor controlMotorBL(&pidLoMotorBL, &smcMotorBL, BASE_MOTOR_V_LIM, BASE_BL_KP, BASE_BL_KP_MAX, BASE_BL_KD, BASE_BL_KD_MAX, BASE_BL_SMC_KP, BASE_BL_SMC_KP_MAX);

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
float v_FL_curr = 0, v_FR_curr = 0, v_BL_curr = 0, v_BR_curr = 0;
float v_FL_real = 0, v_FR_real = 0, v_BL_real = 0, v_BR_real = 0;
uint32_t prevFLticker=0, prevFRticker=0, prevBLticker=0, prevBRticker=0;

/* INITIALIZE TIMERS*/
/* Time Sampling for Base */
uint32_t samplingStick = 0; 
uint32_t samplingPID = 0;
uint32_t samplingOdom = 0;
// uint32_t samplingStickRead = 0;
uint32_t samplingMotor = 0;
// uint32_t samplingUpdPos = 0;
uint32_t samplingEncoder = 0;
uint32_t samplingIK = 0;
// uint32_t samplingPrint = 0;
// uint32_t samplingOtomatis = 0;
// uint32_t samplingOtomatisESP = 0;
// uint32_t samplingParallelPark = 0;
// uint32_t samplingAutoAim = 0;
uint32_t samplingAccel = 0;

uint32_t accelPeriod=250;
#define accelResolution 100.0f

void integReseter(){
    if(v_FL_curr==0){
        pidLoMotorFL.reset();
        // smcMotorFL.reset();
    }
    if(v_FR_curr==0){
        pidLoMotorFR.reset();
        // smcMotorFR.reset();
    }
    if(v_BL_curr==0){
        pidLoMotorBL.reset();
        // smcMotorBL.reset();
    }
    if(v_BR_curr==0){
        pidLoMotorBR.reset();
        // smcMotorBR.reset();
    }
}

void updateEncoder(){
    uint32_t currFLticker = us_ticker_read();
    v_FL_real = (float)encoderBaseBR.getPulses() * 2.0f * PI * WHEEL_RAD * (float)S_TO_US / (ENC_MOTOR_PULSE * (currFLticker - prevFLticker));
    v_FL_curr = movAvg.movingAverage(v_FL_real);
    prevFLticker = currFLticker;
    encoderBaseFL.reset();

    uint32_t currFRticker = us_ticker_read();
    v_FR_real = (float)encoderBaseBR.getPulses() * 2.0f * PI * WHEEL_RAD * (float)S_TO_US / (ENC_MOTOR_PULSE * (currFRticker - prevFRticker));
    v_FR_curr = movAvg.movingAverage(v_FR_real);
    prevFRticker = currFLticker;
    encoderBaseFR.reset();

    uint32_t currBLticker = us_ticker_read();
    v_BL_real = (float)encoderBaseBR.getPulses() * 2.0f * PI * WHEEL_RAD * (float)S_TO_US / (ENC_MOTOR_PULSE * (currBLticker - prevBLticker));
    v_BL_curr = movAvg.movingAverage(v_BL_real);
    prevBLticker = currBLticker;
    encoderBaseBL.reset();

    uint32_t currBRticker = us_ticker_read();
    v_BR_real = (float)encoderBaseBR.getPulses() * 2.0f * PI * WHEEL_RAD * (float)S_TO_US / (ENC_MOTOR_PULSE * (currBRticker - prevBRticker));
    v_BR_curr = movAvg.movingAverage(v_BR_real);
    prevBRticker = currBRticker;
    encoderBaseBR.reset();

    omni.set_v_curr(v_FL_curr, v_FR_curr, v_BL_curr, v_BR_curr);
}

int main(){
    printf("STICK START\n");

    /* SET UP JOYSTICK */
    ps3.idle();   
    ps3.setup();
    omni.encoderMotorSamp(); //baca data awal encoder omniWheel
    while (true){
        // if (serial_port.readable())
        // {
        //     // Parse the received string to extract KP, KI, KD values
        //     scanf("%ld", &accelPeriod);
        //     // Print the updated values
        //     printf("Updated T: T=%d\n", accelPeriod);
        //     // Set the updated PID values
        // }
        
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
        // vx_cmd = 0;
        // vy_cmd = 0;
        // w_cmd = 0;

        //braking system
        if(ps3.getKotak()){ //tombol utk nembak
            omni.forceBrakeSync(); //hard brake sebelum nembak agar robot tidak gerak saat nembak
        } else{
            //moving
            if(us_ticker_read() - samplingAccel > accelPeriod*(1000/(int)accelResolution)){
                if (ps3.getButtonUp()) { //gerak ke atas
                    vy_cmd=vy_cmd+(TRANSLATION_BASE_SPEED/accelResolution);
                    if(vy_cmd>TRANSLATION_BASE_SPEED){
                        vy_cmd = TRANSLATION_BASE_SPEED;
                    }
                } 
                else if (ps3.getButtonDown()) { //gerak ke bawah
                    vy_cmd=vy_cmd-(TRANSLATION_BASE_SPEED/accelResolution);
                    if(vy_cmd< -1.0f * TRANSLATION_BASE_SPEED){
                        vy_cmd = -1.0f * TRANSLATION_BASE_SPEED;
                    }
                }
                else{
                    if(vy_cmd>0.0f){
                        vy_cmd=vy_cmd-(TRANSLATION_BASE_SPEED/accelResolution);
                    } else if(vy_cmd<0.0f){
                        vy_cmd=vy_cmd+(TRANSLATION_BASE_SPEED/accelResolution);
                    }
                    if(vy_cmd> -1.0f * (TRANSLATION_BASE_SPEED/accelResolution) && vy_cmd<(TRANSLATION_BASE_SPEED/accelResolution)){
                        vy_cmd=0.0f;
                        integReseter();
                    }
                }

                if (ps3.getButtonRight()) { //gerak ke atas
                    vx_cmd=vx_cmd+(TRANSLATION_BASE_SPEED/accelResolution);
                    if(vx_cmd>TRANSLATION_BASE_SPEED){
                        vx_cmd = TRANSLATION_BASE_SPEED;
                    }
                } 
                else if (ps3.getButtonLeft()) { //gerak ke bawah
                    vx_cmd=vx_cmd-(TRANSLATION_BASE_SPEED/accelResolution);
                    if(vx_cmd< -1.0f * TRANSLATION_BASE_SPEED){
                        vx_cmd = -1.0f * TRANSLATION_BASE_SPEED;
                    }
                }
                else{
                    if(vx_cmd>0.0f){
                        vx_cmd=vx_cmd-(TRANSLATION_BASE_SPEED/accelResolution);
                    } else if(vx_cmd<0.0f){
                        vx_cmd=vx_cmd+(TRANSLATION_BASE_SPEED/accelResolution);
                    }
                    if(vx_cmd> -1.0f * (TRANSLATION_BASE_SPEED/accelResolution) && vx_cmd<(TRANSLATION_BASE_SPEED/accelResolution)){
                        vx_cmd=0.0f;
                        integReseter();
                    }
                }

                if (ps3.getL1()) { //gerak ke atas
                    w_cmd=w_cmd+(ROTATION_BASE_SPEED/accelResolution);
                    if(w_cmd>ROTATION_BASE_SPEED){
                        w_cmd = ROTATION_BASE_SPEED;
                    }
                } 
                else if (ps3.getR1()) { //gerak ke bawah
                    w_cmd=w_cmd-(ROTATION_BASE_SPEED/accelResolution);
                    if(w_cmd< -1.0f * ROTATION_BASE_SPEED){
                        w_cmd = -1.0f * ROTATION_BASE_SPEED;
                    }
                }
                else{
                    if(w_cmd>0.0f){
                        w_cmd=w_cmd-(ROTATION_BASE_SPEED/accelResolution);
                    } else if(w_cmd<0.0f){
                        w_cmd=w_cmd+(ROTATION_BASE_SPEED/accelResolution);
                    }
                    if(w_cmd> -1.0f * (ROTATION_BASE_SPEED/accelResolution) && w_cmd<(ROTATION_BASE_SPEED/accelResolution)){
                        w_cmd=0.0f;
                        integReseter();
                    }
                }

                omni.set_vx_cmd(vx_cmd);
                omni.set_vy_cmd(vy_cmd);
                omni.set_w_cmd(w_cmd);

                omni.base();

                samplingAccel=us_ticker_read();
            }

            // if (vx_cmd == 0.0 && vy_cmd == 0.0 && w_cmd == 0.0) {
            //     // Change controllers parameters if the base about to stop wholly (soft braking)
            //     smcMotorBL.setKp(BASE_BL_SMC_KP_BRAKE);
            //     smcMotorBR.setKp(BASE_BR_SMC_KP_BRAKE);
            //     smcMotorFL.setKp(BASE_FL_SMC_KP_BRAKE);
            //     smcMotorFR.setKp(BASE_FR_SMC_KP_BRAKE);

            //     smcMotorBL.setKsigma(SMC_BL_KSIGMA_BRAKE);
            //     smcMotorBR.setKsigma(SMC_BR_KSIGMA_BRAKE);
            //     smcMotorFL.setKsigma(SMC_FL_KSIGMA_BRAKE);
            //     smcMotorFR.setKsigma(SMC_FR_KSIGMA_BRAKE);

            //     pidMotorBL.setKp(BASE_BL_KP_BRAKE);
            //     pidMotorBR.setKp(BASE_BR_KP_BRAKE);
            //     pidMotorFL.setKp(BASE_FL_KP_BRAKE);
            //     pidMotorFR.setKp(BASE_FR_KP_BRAKE);    
            // } else {
            //     // Change back controller parameters if base is moving

            //     if (ps3.getR2()) {
            //         // Boost mode, also change controllers parameter
            //         vx_cmd *= TRANSLATION_BOOST_MULTIPLIER;
            //         vy_cmd *= TRANSLATION_BOOST_MULTIPLIER;
            //         w_cmd *= ROTATION_BOOST_MULTIPLIER;

            //         smcMotorBL.setKp(BASE_BL_SMC_KP_BOOST);
            //         smcMotorBR.setKp(BASE_BR_SMC_KP_BOOST);
            //         smcMotorFL.setKp(BASE_FL_SMC_KP_BOOST);
            //         smcMotorFR.setKp(BASE_FR_SMC_KP_BOOST);
            //     } else { //normal mode
            //         smcMotorBL.setKp(BASE_BL_SMC_KP);
            //         smcMotorBR.setKp(BASE_BR_SMC_KP);
            //         smcMotorFL.setKp(BASE_FL_SMC_KP);
            //         smcMotorFR.setKp(BASE_FR_SMC_KP);
            //     }

            //     smcMotorBL.setKsigma(SMC_BL_KSIGMA);
            //     smcMotorBR.setKsigma(SMC_BR_KSIGMA);
            //     smcMotorFL.setKsigma(SMC_FL_KSIGMA);
            //     smcMotorFR.setKsigma(SMC_FR_KSIGMA);

            //     pidMotorBL.setKp(BASE_BL_KP);
            //     pidMotorBR.setKp(BASE_BR_KP);
            //     pidMotorFL.setKp(BASE_FL_KP);
            //     pidMotorFR.setKp(BASE_FR_KP);
            // }



            /********************** ENCODER **********************/

            if (us_ticker_read() - samplingEncoder > SAMP_BASE_MOTOR_ENCODER_US/10)
            {
                
                updateEncoder();
                // omni.getVars(&junk1,&junk1,&junk1,&v_FL_curr,&v_FR_curr,&v_BR_curr,&v_BL_curr,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1);
                printf("vFL: %.2f vFR: %.2f vBL: %.2f vBR: %.2f T: %d\n", v_FL_curr, v_FR_curr, v_BL_curr, v_BR_curr, accelPeriod);
                // printf("FL: %d FR: %d BL:%d BR: %d\n", encoderBaseFL.getPulses(), encoderBaseFR.getPulses(), encoderBaseBL.getPulses(), encoderBaseBR.getPulses());

                samplingEncoder = us_ticker_read();
            }

            /********************** PID **********************/

            if (us_ticker_read() - samplingPID > SAMP_PID_BASE_MOTOR_US) //update PID berdasarkan target dan current speed tiap motor
            {
                omni.pidMotorSamp();

                samplingPID = us_ticker_read();
            }

            /********************** MOTOR **********************/

            if (us_ticker_read() - samplingMotor > SAMP_BASE_MOTOR_US) //update motor
            {
                samplingMotor = us_ticker_read();

                omni.motorSamp();
            }

        }
    }
}