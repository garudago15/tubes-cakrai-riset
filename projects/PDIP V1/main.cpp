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

// -------------------------------------------------------------------------
// UNTUK SHOOTER
#include "Configurations/Constants.h"
#include "Configurations/Setup.h"
#include "ShooterMotor/ShooterMotor.h"
#include "AngleShooter/AngleShooter.h"
#include "../../KRAI_Library/MovingAverage/MovingAverage.h"

// Untuk LED
#include "../../KRAI_Library/led4Pin/led4Pin-digital.h"

// Inisialisasi Pengendali Motor Shooter
ShooterMotor controlShooterMotor(&leftMotor, &encLeftMotor, &pidLeftMotor, &movAvgLM, &movAvgAccel, &motorReload, &encMotorReload);
AngleShooter controlAngShooter(&motorAngle, &encMotorAngle, &pidMotorAngle, &movAvgANG);
JoystickPS3 ps3(UART_STICK_TX, UART_STICK_RX);

// UNTUK LIMIT SWITCH
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
// -----------------------------------------------------------------------

// --------------------- UNTUK LED ------------------------
// // DEFINE LED
// #define red PD_3
// #define green PC_13
// #define blue PC_12
// LED4Pin RGB(red, green, blue);

#define RED             PE_4
#define GREEN           PC_12
#define BLUE            PD_0
LED4PinDigital RGB(RED, GREEN, BLUE);

string state = "notready";
// ---------------------------------------------------------


/* Base Motor */
Motor baseMotorFL(BASE_MOTOR_FL_PWM, BASE_MOTOR_FL_FOR, BASE_MOTOR_FL_REV);
Motor baseMotorFR(BASE_MOTOR_FR_PWM, BASE_MOTOR_FR_FOR, BASE_MOTOR_FR_REV);
Motor baseMotorBL(BASE_MOTOR_BL_PWM, BASE_MOTOR_BL_FOR, BASE_MOTOR_BL_REV);
Motor baseMotorBR(BASE_MOTOR_BR_PWM, BASE_MOTOR_BR_FOR, BASE_MOTOR_BR_REV);

/* PID Vx, Vy, W */
// pidLo vxPid(0, 0, 0, SAMP_IK_US, 10, 1, 1000, 100);
// pidLo vyPid(0, 0, 0, SAMP_IK_US, 10, 1, 1000, 100);
// pidLo wPid(0.025, 0.0025, 0, SAMP_IK_US, 10, 1, 1000, 100);

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
encoderKRAI encoderBaseFL(ENCODER_BASE_MOTOR_FL_CHA, ENCODER_BASE_MOTOR_FL_CHB, PPR_FL / 2, X2_ENCODING);
encoderKRAI encoderBaseFR(ENCODER_BASE_MOTOR_FR_CHA, ENCODER_BASE_MOTOR_FR_CHB, PPR_FR / 2, X2_ENCODING);
encoderKRAI encoderBaseBR(ENCODER_BASE_MOTOR_BR_CHA, ENCODER_BASE_MOTOR_BR_CHB, PPR_BR / 2, X2_ENCODING);
encoderKRAI encoderBaseBL(ENCODER_BASE_MOTOR_BL_CHA, ENCODER_BASE_MOTOR_BL_CHB, PPR_BL / 2, X2_ENCODING);

/* Odometry Encoder */
// encoderKRAI encL(ENCODER_L_CHA, ENCODER_L_CHB, 1000, X4_ENCODING);
// encoderKRAI encAux(ENCODER_AUX_CHA, ENCODER_AUX_CHB, 1000, X4_ENCODING);

/* Odometry */
// odom2enc odom(&encAux, &encL);

/* Trajectory Following */
// StanleyPursuit line;
// pidLo pid(0.08, 0.05, 0.5, 0.5, 0.5, 0, 1000, 100);
// pidLo pid2(0.03, 0.005, 0.2, 0.5, 0.5, 0, 1000, 100);

/* ControlKRAI */
ControlAutomatic4Omni omni(&baseMotorFL, &baseMotorFR, &baseMotorBR, &baseMotorBL, &encoderBaseFL, &encoderBaseFR, &encoderBaseBR, &encoderBaseBL, &controlMotorFL, &controlMotorFR, &controlMotorBR, &controlMotorBL);

//omni variable
float vx_cmd = 0, vy_cmd = 0, w_cmd = 0;
float v_FL_curr = 0, v_FR_curr = 0, v_BL_curr = 0, v_BR_curr = 0;


/* INITIALIZE TIMERS*/
/* Time Sampling for Base */
uint32_t samplingStick = 0; 
uint32_t samplingPIDBase = 0;
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

/* UNTUK PEMBACAAN JARAK DARI NUCLEO */
const int MAX_MSG = 64;
static char message[MAX_MSG];
int constPID_pos = 0;
static unsigned int message_pos = 0;
char inByte;
static float jarakTF[2];


int main(){
    // printf("STICK START\n");
    jarakTF[0] = 0.0;

    /* SET UP JOYSTICK */
    ps3.idle();   
    ps3.setup();

    /* INISIASI LIMIT SWITCH */
    LimitSwitch.rise(&riseMotor);
    LimitSwitch.fall(&fallMotor);

    omni.encoderMotorSamp(); //baca data awal encoder omniWheel

    /* FOR SHOOTER PURPOSE */
    float trySetPoint = 0.0f; // SetPoint PID (RPM)
    int angleSetPoint = 80;
    bool startReload = false;

    /* Debugin Purpose */
    float Kp, Ki, Kd;
    
    char dataSerialC_1;
    float dataSerialF_1, dataSerialF_2;
    int dataSerialI_1;
    // -------------------



    while (true){
        // Untuk Pekerluan LED
        us_timestamp_t currentTimestampLED = us_ticker_read();

        // Pengolahan dan Update data PS3
        ps3.olah_data();
        ps3.baca_data();

        // ------------------------------ BUAT BACA JARAK DARI NUCLEO --------------------

        while (serial_port.readable())
        {
            serial_port.read(&inByte, 1);
            printf("KEBACA \n");

            if ((inByte != '\n') && message_pos < MAX_MSG - 1)
            {
                message[message_pos] = inByte;
                printf("%c\n", inByte);
                message_pos++;
            }
            else
            {
                message[message_pos] = '\0';
                message_pos = 0;
                char *token = strtok(message, " ");

                while (token != NULL)
                {
                    jarakTF[constPID_pos] = atof(token);
                    //printf("%s %d\n", token, constPID_pos);
                    printf("%d\n", constPID_pos);
                    token = strtok(NULL, " ");
                    constPID_pos++;
                    if (constPID_pos == 2)
                    {
                        constPID_pos = 0;
                    }
                }

                // printf("kp = %f || ki = %f || kd = %f\n", pidLeftMotor.getPParam(), pidLeftMotor.getIParam(), pidLeftMotor.getDParam());
                // printf("kp = %f || ki = %f || kd = %f\n", kpKiKd[0], kpKiKd[1], kpKiKd[2]);
            }
        }

        // -------------------------------------------------------------------------------

        // printf("%.2f \n", jarakTF[0]);
        // VALIDASI TF MINI
        // JIka diatas 10m buat jadi 0
        if (jarakTF[0] > 10.0){
            jarakTF[0] = 0.0;
        }

        /* ------------------------ BUAT CEK TF MINI ------------------------------------- */
        // if (jarakTF[0] == 0.0){
        //     RGB.setRGB(false, false, true);
        // } else if ( jarakTF[0] > 0.0 && jarakTF[0] < 10.0) {
        //     RGB.setRGB(false, true, false);
        // } else {
        //     RGB.setRGB(true, true, true);
        // }

        // ----------------------------- SET SETPOINT -------------------------------
        // if (serial_port.readable())
        // {
        //     scanf("%f %d", &dataSerialF_1, &dataSerialI_1);
        //     angleSetPoint = dataSerialI_1;
        //     trySetPoint = dataSerialF_1;

        // }

        // ------------------------- RELOAD MECHANISM -------------------------
        if (ps3.getKotak() && controlShooterMotor.getOmegaShooter() > 1000 &&
        (controlShooterMotor.getAccelShooter() > -5.0f && controlShooterMotor.getAccelShooter() < 5.0f) &&
        (controlShooterMotor.getOmegaShooter() - trySetPoint > -50.0f && controlShooterMotor.getOmegaShooter() - trySetPoint < 50.0f))
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

        // ------------------------- CONTROL ANGLE SHOOTER -------------------------
        
        if (ps3.getLY() < -120)         // Gerakin ke atas
        {
            angleSetPoint = 55;
        } else if (ps3.getLX() < -120)   // Gerakin ke kiri, buat area tengah
        {
            angleSetPoint = 65;
        } else if (ps3.getLY() > 120)  // Gerakin ke bawah, buat area paling deket
        {
            angleSetPoint = 65;
        }

        // printf("%d %d\n", ps3.getLY(), angleSetPoint);       // Buat debug bener ga ni tombol

        // Reset Angle Purpose
        if (ps3.getLY() < -120 && ps3.getRY() < -120)
        {
            RESET_ANG_MOTOR = true;
            angleSetPoint = 80;                                 // Posisi Paling Bawah
        }
        controlAngShooter.resetMotorAngle(0.3, RESET_ANG_MOTOR);

        // Sampling for PID Purpose
        if (us_ticker_read() - timeLastForAngleShooter > samplingPID)
        {
            controlAngShooter.controlAng(angleSetPoint);
            timeLastForAngleShooter = us_ticker_read();
        }
        // ------------------------------------------------------------------------------

        // ------------------------- CONTROL MOTOR SHOOTER -------------------------
        if (ps3.getLingkaran() && ps3.getL2())
        {
            trySetPoint = 0;
        } else if (ps3.getLingkaran() && ps3.getButtonUp())
        {
            trySetPoint = 2950;         // nah ni buat jauh sangat sekitar 3.1m sensor atau 2.5 jarak di asli
        } else if (ps3.getLingkaran() && ps3.getButtonLeft())
        {
            trySetPoint = 2770;         // 2.5m pembacaan sensor berarti 2m jarak di asli
        } else if (ps3.getLingkaran() && ps3.getButtonDown())
        {
            trySetPoint = 1100.0;       // ini buat deket banget sudut 70
        } else if (ps3.getLingkaran() && ps3.getButtonRight())
        {
            trySetPoint = controlShooterMotor.getCalcRPM(jarakTF[0], angleSetPoint);
        }

        // PASTIIN RPM DIBAWAH BATAS MAKSIMUM MOTOR
        if (trySetPoint > 3500.0){
            trySetPoint = 0.0;
        }

        if (us_ticker_read() - timeLast > samplingPID)
        { 
            controlShooterMotor.controlOmegaShooter(trySetPoint);
        }
        // ------------------------------------------------------------------------------
        

        /* UNTUK TUNNING LAPANGAN */
        printf("%f %f %d %d\n", controlShooterMotor.getOmegaShooter(), controlShooterMotor.getSetpoint(), controlAngShooter.getAngleRealtime(), controlAngShooter.getAngleTarget());
        
        // utk reset
        if (ps3.getStart())
        {
            NVIC_SystemReset();
            // printf("start");
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
            if (ps3.getButtonRight() && !ps3.getLingkaran()) { //gerak ke kanan
                vx_cmd += TRANSLATION_BASE_SPEED;
            } 
            if (ps3.getButtonLeft() && !ps3.getLingkaran()) { //gerak ke kiri
                vx_cmd -= TRANSLATION_BASE_SPEED;
            } 

            if (ps3.getButtonUp() && !ps3.getLingkaran()) { //gerak ke atas
                vy_cmd += TRANSLATION_BASE_SPEED;
            } 
            if (ps3.getButtonDown() && !ps3.getLingkaran()) { //gerak ke bawah
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
                // printf("vFL: %.2f vFR: %.2f vBL: %.2f vBR: %.2f\n", omni.get_v_FL_curr(), omni.get_v_FR_curr(), omni.get_v_BL_curr(), omni.get_v_BR_curr());
                // printf("FL: %d FR: %d BL:%d BR: %d\n", encoderBaseFL.getPulses(), encoderBaseFR.getPulses(), encoderBaseBL.getPulses(), encoderBaseBR.getPulses());

                samplingEncoder = us_ticker_read();
            }

            /********************** PID & ODOM **********************/

            if (us_ticker_read() - samplingPIDBase > SAMP_PID_BASE_MOTOR_US) //update PID berdasarkan target dan current speed tiap motor
            {
                omni.pidMotorSamp();

                samplingPIDBase = us_ticker_read();
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


        // --------------------------------- UNTUK CONTROL LED --------------------------
        // APAKAH SEDANG GERAK?
        if (omni.get_v_FL_curr() != 0 || omni.get_v_FR_curr() != 0 || omni.get_v_BL_curr() != 0 || omni.get_v_BR_curr() != 0)
        {
            state = "movement";

        // APAKAH SEDAH RELOAD?
        } else if (startReload) {
            state = "reload";

        // CEK APAKAH SUDAH MENCAPAI SEMUA SETPOINT
        } else if ( ((controlAngShooter.getAngleRealtime() < controlAngShooter.getAngleTarget() + 4) && (controlAngShooter.getAngleRealtime() > controlAngShooter.getAngleTarget() - 4)) &&
                    ((controlShooterMotor.getOmegaShooter() < controlShooterMotor.getSetpoint() + 100) && (controlShooterMotor.getOmegaShooter() > controlShooterMotor.getSetpoint() - 100)) && 
                    (controlShooterMotor.getSetpoint() != 0 && controlAngShooter.getAngleTarget() != 80) )
        {
            state = "ready";
        
        // SEMUA TIDAK READY
        } 
        
        // else if (jarakTF[0] == 0.0 || jarakTF[0] > 4.0){
        //     state = "notf";                 //JArak gakebaca
        // } 
        
        else {
            state = "notready";
        }

        // MODE SELEBRASI
        if(ps3.getSelect()){
            state = "SELEBERASI";
        }


        if (state == "ready"){
            RGB.setColor("GREEN");
        } else if (state == "notready") {
            RGB.setColor("RED");
        } else if (state == "movement") {
            RGB.setColor("BLUE");
        } else if (state == "reload") {
            RGB.setColor("PURPLE");
        } else if (state == "SELEBERASI") {
            RGB.turnOff();
        } 
        
        // else if (state == "notf"){
        //     RGB.setRGB(true, true, true);
        // }
        // ------------------------------------------------------------------------------

        
    }
}