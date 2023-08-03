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

#define Kp_pid_def 1.5f
#define Ki_pid_def 0.01f
#define Kd_pid_def 0.0f
uint32_t accelPeriod=250;
#define Kp_smc_def 0.0028f
#define Ksigma_smc_def 10.0f

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

PID pidMotorFL(Kp_pid_def, Ki_pid_def, Kd_pid_def, BASE_FL_TS_MS, 0.5, 6); //skrg lg tune ini
PID pidMotorFR(BASE_FR_KP, BASE_FR_KI, BASE_FR_KD, BASE_FR_TS_MS, 0.5, 6);
PID pidMotorBR(BASE_BR_KP, BASE_BR_KI, BASE_BR_KD, BASE_BR_TS_MS, 0.5, 6);
PID pidMotorBL(BASE_BL_KP, BASE_BL_KI, BASE_BL_KD, BASE_BL_TS_MS, 0.5, 8);

pidLo pidloMotorFL(Kp_pid_def, Ki_pid_def, Kd_pid_def, BASE_FL_TS_MS, 1.0, 0, 1000, 1000);
pidLo pidloMotorFR(Kp_pid_def, Ki_pid_def, Kd_pid_def, BASE_FL_TS_MS, 1.0, 0, 1000, 1000);
pidLo pidloMotorBL(Kp_pid_def, Ki_pid_def, Kd_pid_def, BASE_FL_TS_MS, 1.0, 0, 1000, 1000);
pidLo pidloMotorBR(Kp_pid_def, Ki_pid_def, Kd_pid_def, BASE_FL_TS_MS, 1.0, 0, 1000, 1000);

/* SMC untuk motor */
SMC smcMotorFL(Kp_smc_def, Ksigma_smc_def, SMC_EPSILON, SMC_BETA, (float)SAMP_PID_BASE_MOTOR_US / MS_TO_US, SMC::KECEPATAN);
SMC smcMotorFR(BASE_FR_SMC_KP, SMC_FR_KSIGMA, SMC_EPSILON, SMC_BETA, (float)SAMP_PID_BASE_MOTOR_US / MS_TO_US, SMC::KECEPATAN);
SMC smcMotorBR(BASE_BR_SMC_KP, SMC_BR_KSIGMA, SMC_EPSILON, SMC_BETA, (float)SAMP_PID_BASE_MOTOR_US / MS_TO_US, SMC::KECEPATAN);
SMC smcMotorBL(BASE_BL_SMC_KP, SMC_BL_KSIGMA, SMC_EPSILON, SMC_BETA, (float)SAMP_PID_BASE_MOTOR_US / MS_TO_US, SMC::KECEPATAN);

/* Control Motor */
ControlMotor controlMotorFL(&pidMotorFL, &smcMotorFL, BASE_MOTOR_V_LIM, Kp_pid_def, BASE_FL_KP_MAX, Kd_pid_def, BASE_FL_KD_MAX, Kp_smc_def, BASE_FL_SMC_KP_MAX);
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
float v_FL_curr = 0, v_FR_curr = 0, v_BL_curr = 0, v_BR_curr = 0;
float v_FL_real = 0, v_FR_real = 0, v_BL_real = 0, v_BR_real = 0;


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

//PID variables
float Kp_pid, Ki, Kd, Kp_smc, Ksigma;
float curr_pwm=0.0f;
float last_vy_cmd;

uint32_t prevFLticker=0, prevFRticker=0, prevBLticker=0, prevBRticker=0;
uint32_t last_current=0;


void pid_test(){
    printf("STICK START\n");

    /* SET UP JOYSTICK */
    ps3.idle();   
    ps3.setup();
    omni.encoderMotorSamp(); //baca data awal encoder omniWheel
    while (true){
        if (serial_port.readable())
        {
            // Parse the received string to extract KP, KI, KD values
            scanf("%f %f %f %ld", &Kp_pid, &Ki, &Kd, &accelPeriod);
            // Print the updated values
            printf("Updated PID values: Kp=%.2f, Ki=%.2f, Kd=%.2f, T=%d\n", Kp_pid, Ki, Kd, accelPeriod);
            // Set the updated PID values
            pidloMotorFR.setTunings(Kp_pid, Ki, Kd);

            // scanf("%f", &motor_default_speed);
        }
        

        // Pengolahan dan Update data PS3
        ps3.olah_data();
        ps3.baca_data();

        //default
        // vy_cmd = 0;

        //braking system
        if(ps3.getKotak()){ //tombol utk nembak
            omni.forceBrakeSync(); //hard brake sebelum nembak agar robot tidak gerak saat nembak
        } else{
            if(us_ticker_read() - samplingAccel > accelPeriod){
                if (ps3.getButtonUp()) { //gerak ke atas
                    vy_cmd=vy_cmd+0.006f;
                    if(vy_cmd>0.636f){
                        vy_cmd = 0.636f;
                    }
                } 
                else if (ps3.getButtonDown()) { //gerak ke bawah
                    vy_cmd=vy_cmd-0.006f;
                    if(vy_cmd<-0.636f){
                        vy_cmd = -0.636f;
                    }
                }
                else{
                    if(vy_cmd>0.0f){
                        vy_cmd=vy_cmd-0.006f;
                    } else if(vy_cmd<0.0f){
                        vy_cmd=vy_cmd+0.006f;
                    }
                    if(vy_cmd>-0.005f && vy_cmd<0.005f){
                        vy_cmd=0.0f;
                        if(v_FR_curr==0){
                            pidloMotorFR.reset();
                        }
                    }
                }
                samplingAccel=us_ticker_read();

            }

            /********************** ENCODER **********************/

            if (us_ticker_read() - samplingEncoder > SAMP_BASE_MOTOR_ENCODER_US/10)
            {
                // omni.encoderMotorSamp();
                uint32_t currFRticker = us_ticker_read();
                v_FR_real = (float)encoderBaseFR.getPulses() * 2.0f * PI * WHEEL_RAD * (float)S_TO_US / (ENC_MOTOR_PULSE * (currFRticker - prevFRticker));
                v_FR_curr = movAvg.movingAverage(v_FR_real);
                prevFRticker = currFRticker;
                encoderBaseFR.reset();
                
                // omni.getVars(&junk1,&junk1,&junk1,&v_FL_curr,&v_FR_curr,&v_BR_curr,&v_BL_curr,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1);
                printf("goal: %.2f vFR: %.2f pwm: %.2f p: %.5f i: %.5f d: %.5f T: %d\n", vy_cmd, v_FR_curr, curr_pwm, Kp_pid, Ki, Kd, accelPeriod);
                // printf("FL: %d FR: %d BL:%d BR: %d\n", encoderBaseFL.getPulses(), encoderBaseFR.getPulses(), encoderBaseBL.getPulses(), encoderBaseBR.getPulses());

                samplingEncoder = us_ticker_read();
            }

            /********************** PID & ODOM **********************/

            if (us_ticker_read() - samplingPID > SAMP_PID_BASE_MOTOR_US) //update PID berdasarkan target dan current speed tiap motor
            {
                curr_pwm=pidloMotorFR.createpwm(vy_cmd,v_FR_curr,0.8f);
                // last_vy_cmd=vy_cmd;

                samplingPID = us_ticker_read();
            }


            /********************** MOTOR **********************/

            if (us_ticker_read() - samplingMotor > SAMP_BASE_MOTOR_US) //update motor
            {
                samplingMotor = us_ticker_read();

                baseMotorFR.speed(curr_pwm);
                // baseMotorFR.speed(0.3);
            }

        }
    }
}

void smc_test(){
    printf("STICK START\n");

    /* SET UP JOYSTICK */
    ps3.idle();   
    ps3.setup();
    omni.encoderMotorSamp(); //baca data awal encoder omniWheel
    while (true){
        if (serial_port.readable())
        {
            // Parse the received string to extract KP, KI, KD values
            scanf("%f %f", &Kp_smc, &Ksigma);
            // Print the updated values
            printf("Updated PID values: Kp=%.2f, Ksigma=%.2f\n", Kp_smc, Ksigma);
            // Set the updated PID values
            smcMotorFL.setKp(Kp_smc);
            smcMotorFL.setKsigma(Ksigma);

            // scanf("%f", &motor_default_speed);
        }
        
        // Pengolahan dan Update data PS3
        ps3.olah_data();
        ps3.baca_data();

        //default
        vy_cmd = 0;

        //braking system
        if(ps3.getKotak()){ //tombol utk nembak
            omni.forceBrakeSync(); //hard brake sebelum nembak agar robot tidak gerak saat nembak
        } else{
            if (ps3.getButtonUp()) { //gerak ke atas
                vy_cmd += 0.636f;
            } 
            if (ps3.getButtonDown()) { //gerak ke bawah
                vy_cmd -= 0.636f;
            }

            /********************** ENCODER **********************/

            if (us_ticker_read() - samplingEncoder > SAMP_BASE_MOTOR_ENCODER_US/10)
            {
                // omni.encoderMotorSamp();
                uint32_t currFLticker = us_ticker_read();
                v_FL_real = (float)encoderBaseFL.getPulses() * 2.0f * PI * WHEEL_RAD * (float)S_TO_US / (ENC_MOTOR_PULSE * (currFLticker - prevFLticker));
                v_FL_curr = movAvg.movingAverage(v_FL_real);
                prevFLticker = currFLticker;
                encoderBaseFL.reset();
                
                // omni.getVars(&junk1,&junk1,&junk1,&v_FL_curr,&v_FR_curr,&v_BR_curr,&v_BL_curr,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1);
                printf("goal: %.2f vFR: %.2f pwm: %.2f p: %.5f c: %.5f\n", vy_cmd, v_FL_curr, curr_pwm, Kp_smc, Ksigma);
                // printf("FL: %d FR: %d BL:%d BR: %d\n", encoderBaseFL.getPulses(), encoderBaseFR.getPulses(), encoderBaseBL.getPulses(), encoderBaseBR.getPulses());

                samplingEncoder = us_ticker_read();
            }

            /********************** PID & ODOM **********************/

            if (us_ticker_read() - samplingPID > SAMP_PID_BASE_MOTOR_US) //update PID berdasarkan target dan current speed tiap motor
            {
                curr_pwm=smcMotorFL.createpwm(vy_cmd,v_FL_curr,0.8f);

                samplingPID = us_ticker_read();
            }


            /********************** MOTOR **********************/

            if (us_ticker_read() - samplingMotor > SAMP_BASE_MOTOR_US) //update motor
            {
                samplingMotor = us_ticker_read();

                baseMotorFL.speed(curr_pwm);
            }

        }
    }
}

void controlMotor_test(){
    printf("STICK START\n");

    /* SET UP JOYSTICK */
    ps3.idle();   
    ps3.setup();
    omni.encoderMotorSamp(); //baca data awal encoder omniWheel
    while (true){
        if (serial_port.readable())
        {
            // Parse the received string to extract KP, KI, KD values
            scanf("%f %f %f %f %f", &Kp_pid, &Ki, &Kd, &Kp_smc, &Ksigma);
            // Print the updated values
            printf("Updated PID values: Kp_pid=%.2f, Ki=%.2f, Kd=%.2f, Kp_smc=%.2f, Ksigma=%.2f\n", Kp_pid, Ki, Kd, Kp_smc, Ksigma);
            // Set the updated PID values
            pidMotorFL.setTunings(Kp_pid, Ki, Kd);
            smcMotorFL.setKp(Kp_smc);
            smcMotorFL.setKsigma(Ksigma);

            // scanf("%f", &motor_default_speed);
        }
        
        // Pengolahan dan Update data PS3
        ps3.olah_data();
        ps3.baca_data();

        //default
        vy_cmd = 0;

        //braking system
        if(ps3.getKotak()){ //tombol utk nembak
            omni.forceBrakeSync(); //hard brake sebelum nembak agar robot tidak gerak saat nembak
        } else{
            if (ps3.getButtonUp()) { //gerak ke atas
                vy_cmd += 0.636f;
            } 
            if (ps3.getButtonDown()) { //gerak ke bawah
                vy_cmd -= 0.636f;
            }

            /********************** ENCODER **********************/

            if (us_ticker_read() - samplingEncoder > SAMP_BASE_MOTOR_ENCODER_US/10)
            {
                omni.encoderMotorSamp();
                
                
                // omni.getVars(&junk1,&junk1,&junk1,&v_FL_curr,&v_FR_curr,&v_BR_curr,&v_BL_curr,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1);
                printf("goal: %.2f vFL: %.2f pwm: %.2f\n", vy_cmd, omni.get_v_FL_curr(), curr_pwm);
                // printf("FL: %d FR: %d BL:%d BR: %d\n", encoderBaseFL.getPulses(), encoderBaseFR.getPulses(), encoderBaseBL.getPulses(), encoderBaseBR.getPulses());

                samplingEncoder = us_ticker_read();
            }

            /********************** PID & ODOM **********************/

            if (us_ticker_read() - samplingPID > SAMP_PID_BASE_MOTOR_US) //update PID berdasarkan target dan current speed tiap motor
            {
                if(last_vy_cmd!=vy_cmd){

                }
                curr_pwm=controlMotorFL.createpwm(vy_cmd,omni.get_v_FL_curr(), 0.8f);
                last_vy_cmd=vy_cmd;

                samplingPID = us_ticker_read();
            }


            /********************** MOTOR **********************/

            if (us_ticker_read() - samplingMotor > SAMP_BASE_MOTOR_US) //update motor
            {
                samplingMotor = us_ticker_read();

                baseMotorFL.speed(curr_pwm);
            }

        }
    }
}

uint32_t samplingTime=SAMP_BASE_MOTOR_ENCODER_US;

void cek_encoder(){
    while(1){
        if (serial_port.readable())
        {
            scanf("%ld", &samplingTime);
        }
        
        if (us_ticker_read() - samplingEncoder > samplingTime/10)
        {
            // omni.encoderMotorSamp();
            uint32_t currFLticker = us_ticker_read();
            v_FL_real = (float)encoderBaseFL.getPulses() * 2.0f * PI * WHEEL_RAD * (float)S_TO_US / (ENC_MOTOR_PULSE * (currFLticker - prevFLticker));
            v_FL_curr = movAvg.movingAverage(v_FL_real);
            prevFLticker = currFLticker;
            encoderBaseFL.reset();
            
            // omni.getVars(&junk1,&junk1,&junk1,&v_FL_curr,&v_FR_curr,&v_BR_curr,&v_BL_curr,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1);
            printf("goal: %.2f vFL: %.2f pwm: %.2f\n", vy_cmd, v_FL_curr, curr_pwm);
            // printf("FL: %d FR: %d BL:%d BR: %d\n", encoderBaseFL.getPulses(), encoderBaseFR.getPulses(), encoderBaseBL.getPulses(), encoderBaseBR.getPulses());

            samplingEncoder = us_ticker_read();
        }
    }
}

int main(){
    pid_test();
    // smc_test();
    // controlMotor_test();
    // cek_encoder();
}