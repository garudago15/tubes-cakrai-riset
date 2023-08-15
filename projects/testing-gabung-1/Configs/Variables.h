#ifndef TESTING_GABUNG_1_VARIABLES
#define TESTING_GABUNG_1_VARIABLES

#include "mbed.h"
#include "../../../libs/Configs/Constants.h"
#include "../../../libs/Configs/ConfigurationPin.h"
// #include "../../../libs/Control4Roda/Control4Omni.h"
#include "../../../KRAI_Library/Motor/Motor.h"
#include "../../../KRAI_Library/pidLo/pidLo.h"
#include "../../../KRAI_Library/SMC_KRAI/SMC_KRAI.h"
#include "../../../KRAI_Library/ControlMotor/ControlMotor.h"
#include "../../../KRAI_Library/encoderHAL/encoderHAL.h"
#include "../../../KRAI_Library/encoderKRAI/encoderKRAI.h"
#include "../../../KRAI_Library/Control4Roda/ControlAutomatic4Omni.h"
#include "../../../KRAI_Library/StanleyPursuit/StanleyPursuit.h"
#include "../../../KRAI_Library/encoderHAL/EncoderMspInitF4.h"
#include "../../../KRAI_Library/odom2enc/Coordinate.h"
#include "../../../KRAI_Library/odom2enc/odom2enc.h"
// #include "../../../KRAI_Library/odom3enc/odom3enc.h"
#include "../../../KRAI_Library/JoystickPS3/JoystickPS3.h"
#include "../../../KRAI_Library/PID_KRAI/PID_KRAI.h"

/* INITIALIZE SERIAL USING PRINTF */
static BufferedSerial serial_port(PA_9, PA_10, 115200);
FileHandle *mbed::mbed_override_console(int fd){
    return &serial_port;
}

/* INITIALIZE PS3 JOYSTICK */
JoystickPS3 ps3(UART_STICK_TX, UART_STICK_RX);

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

//---------------------------------------------------------------------------------------------------------------------
/* VARIABEL SHOOTER*/


//-------------------------------------------------------------------------------------------------------------------------
/*multithreading*/
Thread threadBase;
Thread threadStick;
Thread threadShooter;


#endif