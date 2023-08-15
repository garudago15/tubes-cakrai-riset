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

float Kp_FL=2.5f, Ki_FL=0.01f, Kd_FL=0.0f;
float Kp_FR=3.0f, Ki_FR=0.02f, Kd_FR=0.0f;
float Kp_BL=3.0f, Ki_BL=0.01f, Kd_BL=0.03f;
float Kp_BR=2.5f, Ki_BR=0.02f, Kd_BR=0.0f;

/* INITIALIZE SERIAL USING PRINTF */
static BufferedSerial serial_port(PA_9, PA_10, 115200);
FileHandle *mbed::mbed_override_console(int fd){
    return &serial_port;
}

/* INITIALIZE PS3 JOYSTICK */
JoystickPS3 ps3(UART_STICK_TX, UART_STICK_RX);

/*initialize moving average*/
MovingAverage movAvgFL(10);
MovingAverage movAvgFR(10);
MovingAverage movAvgBL(10);
MovingAverage movAvgBR(10);

/* Base Motor */
Motor baseMotorFL(BASE_MOTOR_FL_PWM, BASE_MOTOR_FL_FOR, BASE_MOTOR_FL_REV);
Motor baseMotorFR(BASE_MOTOR_FR_PWM, BASE_MOTOR_FR_FOR, BASE_MOTOR_FR_REV);
Motor baseMotorBL(BASE_MOTOR_BL_PWM, BASE_MOTOR_BL_FOR, BASE_MOTOR_BL_REV);
Motor baseMotorBR(BASE_MOTOR_BR_PWM, BASE_MOTOR_BR_FOR, BASE_MOTOR_BR_REV);

/* PID Vx, Vy, W */
pidLo vxPid(0, 0, 0, SAMP_IK_US, 10, 1, 1000, 100);
pidLo vyPid(0, 0, 0, SAMP_IK_US, 10, 1, 1000, 100);
pidLo wPid(0.025, 0.0025, 0, SAMP_IK_US, 10, 1, 1000, 100);

pidLo pidLoMotorFL(Kp_FL, Ki_FL, Kd_FL, BASE_FL_TS_MS, 1.0, 0, 1000, 1000);
pidLo pidLoMotorFR(Kp_FR, Ki_FR, Kd_FR, BASE_FL_TS_MS, 1.0, 0, 1000, 1000);
pidLo pidLoMotorBL(Kp_BL, Ki_BL, Kd_BL, BASE_FL_TS_MS, 1.0, 0, 1000, 1000);
pidLo pidLoMotorBR(Kp_BR, Ki_BR, Kd_BR, BASE_FL_TS_MS, 1.0, 0, 1000, 1000);

/* SMC untuk motor */
SMC smcMotorFL(0, 0, SMC_EPSILON, SMC_BETA, (float)SAMP_PID_BASE_MOTOR_US / MS_TO_US, SMC::KECEPATAN);
SMC smcMotorFR(0, 0, SMC_EPSILON, SMC_BETA, (float)SAMP_PID_BASE_MOTOR_US / MS_TO_US, SMC::KECEPATAN);
SMC smcMotorBR(0, 0, SMC_EPSILON, SMC_BETA, (float)SAMP_PID_BASE_MOTOR_US / MS_TO_US, SMC::KECEPATAN);
SMC smcMotorBL(0, 0, SMC_EPSILON, SMC_BETA, (float)SAMP_PID_BASE_MOTOR_US / MS_TO_US, SMC::KECEPATAN);

/* Control Motor */
ControlMotor controlMotorFL(&pidLoMotorFL, &smcMotorFL, BASE_MOTOR_V_LIM, Kp_FL, BASE_FL_KP_MAX, Kd_FL, BASE_FL_KD_MAX, 0.0f, BASE_FL_SMC_KP_MAX);
ControlMotor controlMotorFR(&pidLoMotorFR, &smcMotorFR, BASE_MOTOR_V_LIM, Kp_FR, BASE_FR_KP_MAX, Kd_FR, BASE_FR_KD_MAX, 0.0f, BASE_FR_SMC_KP_MAX);
ControlMotor controlMotorBL(&pidLoMotorBL, &smcMotorBL, BASE_MOTOR_V_LIM, Kp_BL, BASE_BL_KP_MAX, Kd_BL, BASE_BL_KD_MAX, 0.0f, BASE_BL_SMC_KP_MAX);
ControlMotor controlMotorBR(&pidLoMotorBR, &smcMotorBR, BASE_MOTOR_V_LIM, Kp_BR, BASE_BR_KP_MAX, Kd_BR, BASE_BR_KD_MAX, 0.0f, BASE_BR_SMC_KP_MAX);

/* Base Motor Encoder */
encoderKRAI encoderBaseFL(ENCODER_BASE_MOTOR_FL_CHA, ENCODER_BASE_MOTOR_FL_CHB, PPR_FL / 2, X2_ENCODING);
encoderKRAI encoderBaseFR(ENCODER_BASE_MOTOR_FR_CHA, ENCODER_BASE_MOTOR_FR_CHB, PPR_FR / 2, X2_ENCODING);
encoderKRAI encoderBaseBR(ENCODER_BASE_MOTOR_BR_CHA, ENCODER_BASE_MOTOR_BR_CHB, PPR_BR / 2, X2_ENCODING);
encoderKRAI encoderBaseBL(ENCODER_BASE_MOTOR_BL_CHA, ENCODER_BASE_MOTOR_BL_CHB, PPR_BL / 2, X2_ENCODING);

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

float Kp_pid=0, Ki=0, Kd=0;
int tuningMode=0;

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
    v_FL_real = (float)encoderBaseFL.getPulses() * 2.0f * PI * WHEEL_RAD * (float)S_TO_US / (ENC_MOTOR_PULSE * (currFLticker - prevFLticker));
    v_FL_curr = movAvgFL.movingAverage(v_FL_real);
    prevFLticker = currFLticker;
    encoderBaseFL.reset();

    uint32_t currFRticker = us_ticker_read();
    v_FR_real = (float)encoderBaseFR.getPulses() * 2.0f * PI * WHEEL_RAD * (float)S_TO_US / (ENC_MOTOR_PULSE * (currFRticker - prevFRticker));
    v_FR_curr = movAvgFR.movingAverage(v_FR_real);
    prevFRticker = currFLticker;
    encoderBaseFR.reset();

    uint32_t currBLticker = us_ticker_read();
    v_BL_real = (float)encoderBaseBL.getPulses() * 2.0f * PI * WHEEL_RAD * (float)S_TO_US / (ENC_MOTOR_PULSE * (currBLticker - prevBLticker));
    v_BL_curr = movAvgBL.movingAverage(v_BL_real);
    prevBLticker = currBLticker;
    encoderBaseBL.reset();

    uint32_t currBRticker = us_ticker_read();
    v_BR_real = (float)encoderBaseBR.getPulses() * 2.0f * PI * WHEEL_RAD * (float)S_TO_US / (ENC_MOTOR_PULSE * (currBRticker - prevBRticker));
    v_BR_curr = movAvgBR.movingAverage(v_BR_real);
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
        if (serial_port.readable())
        {
            switch(tuningMode){
                case 0:
                    scanf("%d", &tuningMode);
                    break;
                case 1:
                    scanf("%f %f %f", &Kp_FL, &Ki_FL, &Kd_FL);
                    printf("Updated PID FL values: Kp=%.2f, Ki=%.2f, Kd=%.2f\n", Kp_FL, Ki_FL, Kd_FL);
                    pidLoMotorFL.setTunings(Kp_FL, Ki_FL, Kd_FL);
                    break;
                case 2:
                    scanf("%f %f %f", &Kp_FR, &Ki_FR, &Kd_FR);
                    printf("Updated PID FR values: Kp=%.2f, Ki=%.2f, Kd=%.2f\n", Kp_FR, Ki_FR, Kd_FR);
                    pidLoMotorFR.setTunings(Kp_FR, Ki_FR, Kd_FR);
                    break;
                case 3:
                    scanf("%f %f %f", &Kp_BL, &Ki_BL, &Kd_BL);
                    printf("Updated PID BL values: Kp=%.2f, Ki=%.2f, Kd=%.2f\n", Kp_BL, Ki_BL, Kd_BL);
                    pidLoMotorBL.setTunings(Kp_BL, Ki_BL, Kd_BL);
                    break;
                case 4:
                    scanf("%f %f %f", &Kp_BR, &Ki_BR, &Kd_BR);
                    printf("Updated PID BR values: Kp=%.2f, Ki=%.2f, Kd=%.2f\n", Kp_BR, Ki_BR, Kd_BR);
                    pidLoMotorBR.setTunings(Kp_BR, Ki_BR, Kd_BR);
                    break;
                case 5:
                    scanf("%ld", &accelPeriod);
                    printf("Updated accelPeriod: %d\n", accelPeriod);
                default:
                    tuningMode=0;
                    break;
            }
            // Parse the received string to extract KP, KI, KD values
            // scanf("%f %f %f %ld", &Kp_pid, &Ki, &Kd, &accelPeriod);
            // // Print the updated values
            // printf("Updated PID values: Kp=%.2f, Ki=%.2f, Kd=%.2f, T=%d\n", Kp_pid, Ki, Kd, accelPeriod);
            // // Set the updated PID values
            // pidLoMotorFL.setTunings(Kp_pid, Ki, Kd);

            // scanf("%f", &motor_default_speed);
        }
        
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

                omni.basePidLo();

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

            if (us_ticker_read() - samplingEncoder > SAMP_BASE_MOTOR_ENCODER_US)
            {
                
                updateEncoder();
                // omni.getVars(&junk1,&junk1,&junk1,&v_FL_curr,&v_FR_curr,&v_BR_curr,&v_BL_curr,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1);
                // printf("vFL: %.2f vFR: %.2f vBL: %.2f vBR: %.2f T: %d\n", v_FL_curr, v_FR_curr, v_BL_curr, v_BR_curr, accelPeriod);
                // printf("FL: %d FR: %d BL:%d BR: %d\n", encoderBaseFL.getPulses(), encoderBaseFR.getPulses(), encoderBaseBL.getPulses(), encoderBaseBR.getPulses());

                samplingEncoder = us_ticker_read();
            }

            /********************** PID **********************/

            if (us_ticker_read() - samplingPID > SAMP_PID_BASE_MOTOR_US) //update PID berdasarkan target dan current speed tiap motor
            {
                omni.pidLoMotorSamp();
                switch(tuningMode){
                    case 0:
                        printf("vFL: %.2f vFR: %.2f vBL: %.2f vBR: %.2f T: %d\n", v_FL_curr, v_FR_curr, v_BL_curr, v_BR_curr, accelPeriod);
                        break;
                    case 1:
                        printf("goal: %.2f vFL: %.2f pwm: %.2f p: %.5f i: %.5f d: %.5f\n", omni.get_FL_target_speed(), v_FL_curr, omni.get_FL_pwm(), Kp_FL, Ki_FL, Kd_FL); //tuning FL
                        break;
                    case 2:
                        printf("goal: %.2f vFR: %.2f pwm: %.2f p: %.5f i: %.5f d: %.5f\n", omni.get_FR_target_speed(), v_FR_curr, omni.get_FR_pwm(), Kp_FR, Ki_FR, Kd_FR); //tuning FR
                        break;
                    case 3:
                        printf("goal: %.2f vBL: %.2f pwm: %.2f p: %.5f i: %.5f d: %.5f\n", omni.get_BL_target_speed(), v_BL_curr, omni.get_BL_pwm(), Kp_BL, Ki_BL, Kd_BL); //tuning BL
                        break;
                    case 4:
                        printf("goal: %.2f vBR: %.2f pwm: %.2f p: %.5f i: %.5f d: %.5f\n", omni.get_BR_target_speed(), v_BR_curr, omni.get_BR_pwm(), Kp_BR, Ki_BR, Kd_BR); //tuning BR
                        break;
                    case 5:
                        printf("accelPeriod: %d\n", accelPeriod); //tuning T
                        break;
                    default:
                        printf("no mode");
                        break;
                }
                // printf("vFL: %.2f vFR: %.2f vBL: %.2f vBR: %.2f T: %d\n", v_FL_curr, v_FR_curr, v_BL_curr, v_BR_curr, accelPeriod);
                // printf("goal: %.2f vFL: %.2f pwm: %.2f p: %.5f i: %.5f d: %.5f T: %d\n", omni.get_FL_target_speed(), v_FL_curr, omni.get_FL_pwm(), Kp_pid, Ki, Kd, accelPeriod); //tuning FL

                samplingPID = us_ticker_read();
            }

            /********************** MOTOR **********************/

            if (us_ticker_read() - samplingMotor > SAMP_BASE_MOTOR_US) //update motor
            {
                samplingMotor = us_ticker_read();

                omni.motorSamp();
                // baseMotorFL.speed(omni.get_FL_pwm()); //tuning FL
            }

        }
    }
}