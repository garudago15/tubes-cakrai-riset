#ifndef _VARIABLES_H_
#define _VARIABLES_H_

// #include "../mbed_config.h"
#include "rtos.h"
#include "ConfigurationPin.h"
#include "Constants.h"
#include "../../KRAI_Library/Motor/Motor.h"
#include "../../KRAI_Library/pidLo/pidLo.h"
#include "../../KRAI_Library/SMC_KRAI/SMC_KRAI.h"
#include "../../KRAI_Library/ControlMotor/ControlMotor.h"
#include "../../KRAI_Library/encoderHAL/encoderHAL.h"
#include "../../KRAI_Library/encoderKRAI/encoderKRAI.h"
#include "../../KRAI_Library/Control4Roda/ControlAutomatic4Omni.h"
#include "../../KRAI_Library/StanleyPursuit/StanleyPursuit.h"
#include "SPI.h"
#include "../../KRAI_Library/encoderHAL/EncoderMspInitF4.h"
#include "../../KRAI_Library/odom2enc/Coordinate.h"
#include "../../KRAI_Library/odom3enc/odom3enc.h"
#include "../../KRAI_Library/JoystickPS3/JoystickPS3.h"
#include "../../KRAI_Library/pneumaticKRAI/pneumaticKRAI.h"
#include "../../KRAI_Library/PID_KRAI/PID_KRAI.h"
#include "../../KRAI_Library/PIDAaronBerk/PIDAaronBerk.h"
#include "../../KRAI_Library/servoKRAI/servoKRAI.h"
#include "../../KRAI_Library/INA219/INA219.h"
#include "../Shooter/Shooter.h"
// #include "../Shooter/ThrowingMode.h"
#include "../Reloader/Reloader.h"

/*****************************************************************************/
/*                               BASE VARIABLE                               */
/*****************************************************************************/
/* Time Sampling for Base */
uint32_t samplingPID = 0;
uint32_t samplingOdom = 0;
uint32_t samplingStick = 0; // change to samplingStickSetVel
uint32_t samplingStickRead = 0;
uint32_t samplingMotor = 0;
uint32_t samplingUpdPos = 0;
uint32_t samplingEncoder = 0;
uint32_t samplingIK = 0;
uint32_t samplingPrint = 0;
uint32_t samplingOtomatis = 0;
uint32_t samplingOtomatisESP = 0;
uint32_t samplingParallelPark = 0;
uint32_t samplingAutoAim = 0;

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

// PIDAaronBerk pidAaronBerkMotorFL(BASE_FL_KC, BASE_FL_TAUI, BASE_FL_TAUD, BASE_FL_TS);
// PIDAaronBerk pidAaronBerkMotorFR(BASE_FR_KC, BASE_FR_TAUI, BASE_FR_TAUD, BASE_FR_TS);
// PIDAaronBerk pidAaronBerkMotorBR(BASE_BR_KC, BASE_BR_TAUI, BASE_BR_TAUD, BASE_BR_TS);
// PIDAaronBerk pidAaronBerkMotorBL(BASE_BL_KC, BASE_BL_TAUI, BASE_BL_TAUD, BASE_BL_TS);

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

// ControlMotor controlMotorFL(&pidAaronBerkMotorFL, &smcMotorFL, BASE_MOTOR_V_LIM, BASE_FL_KC, BASE_FL_KC_STEADY, BASE_FL_SMC_KP, BASE_FL_SMC_KP_MAX);
// ControlMotor controlMotorFR(&pidAaronBerkMotorFR, &smcMotorFR, BASE_MOTOR_V_LIM, BASE_FR_KC, BASE_FR_KC_STEADY, BASE_FR_SMC_KP, BASE_FR_SMC_KP_MAX);
// ControlMotor controlMotorBR(&pidAaronBerkMotorBR, &smcMotorBR, BASE_MOTOR_V_LIM, BASE_BR_KC, BASE_BR_KC_STEADY, BASE_BR_SMC_KP, BASE_BR_SMC_KP_MAX);
// ControlMotor controlMotorBL(&pidAaronBerkMotorBL, &smcMotorBL, BASE_MOTOR_V_LIM, BASE_BL_KC, BASE_BL_KC_STEADY, BASE_BL_SMC_KP, BASE_BL_SMC_KP_MAX);

/* Base Motor Encoder */
encoderKRAI encoderBaseFL(ENCODER_BASE_MOTOR_FL_CHA, ENCODER_BASE_MOTOR_FL_CHB, ENC_BASE_MOTOR_PPR / 2, X2_ENCODING);
encoderKRAI encoderBaseFR(ENCODER_BASE_MOTOR_FR_CHA, ENCODER_BASE_MOTOR_FR_CHB, ENC_BASE_MOTOR_PPR / 2, X2_ENCODING);
encoderKRAI encoderBaseBR(ENCODER_BASE_MOTOR_BR_CHA, ENCODER_BASE_MOTOR_BR_CHB, ENC_BASE_MOTOR_PPR / 2, X2_ENCODING);
encoderKRAI encoderBaseBL(ENCODER_BASE_MOTOR_BL_CHA, ENCODER_BASE_MOTOR_BL_CHB, ENC_BASE_MOTOR_PPR / 2, X2_ENCODING);

/* Odometry Encoder */
encoderKRAI encL(ENCODER_L_CHA, ENCODER_L_CHB, 1000, X4_ENCODING);
encoderKRAI encAux(ENCODER_AUX_CHA, ENCODER_AUX_CHB, 1000, X4_ENCODING);

/* Joystick */
JoystickPS3 stick(UART_STICK_TX, UART_STICK_RX);

/* Odometry */
odom2enc odom(&encAux, &encL);

/* Trajectory Following */
StanleyPursuit line;
pidLo pid(0.08, 0.05, 0.5, 0.5, 0.5, 0, 1000, 100);
pidLo pid2(0.03, 0.005, 0.2, 0.5, 0.5, 0, 1000, 100);

/* ControlKRAI */
ControlAutomatic4Omni controlElephant(&baseMotorFL, &baseMotorFR, &baseMotorBR, &baseMotorBL, &encoderBaseFL, &encoderBaseFR, &encoderBaseBR, &encoderBaseBL, &controlMotorFL, &controlMotorFR, &controlMotorBR, &controlMotorBL, &odom, &vxPid, &vyPid, &wPid, &line, &pid, &pid2);

/* Serial Port */
static BufferedSerial serial_port(UART_SERIAL_TX, UART_SERIAL_RX, 115200);
FileHandle *mbed::mbed_override_console(int fd)
{
    return &serial_port;
}

// Variabel Kecepatan Base
float vx_cmd = 0, vy_cmd = 0, w_cmd = 0;

/*****************************************************************************/
/*                                SLAVE VARIABLES                            */
/*****************************************************************************/
Motor topRoller(TOP_ROLLER_PWM, TOP_ROLLER_FWD, TOP_ROLLER_REV);
Motor bottomRoller(BOTTOM_ROLLER_PWM, BOTTOM_ROLLER_FWD, BOTTOM_ROLLER_REV);
Motor angleMotor(ANGLE_MOTOR_PWM, ANGLE_MOTOR_FWD, ANGLE_MOTOR_REV);

pneumaticKRAI ringPusher(RING_PUSHER_PIN);

encoderKRAI encoderTopThrower(ENCODER_TOP_ROLLER_PIN_A, ENCODER_TOP_ROLLER_PIN_B, 10332, X4_ENCODING);
encoderKRAI encoderBottomThrower(ENCODER_BOTTOM_ROLLER_PIN_A, ENCODER_BOTTOM_ROLLER_PIN_B, 10332, X4_ENCODING);

INA219 encoderAngleMotor(INA_SDA, INA_SCL, 0x80);

PIDAaronBerk pidBottomThrower(BOTTOM_ROLLER_TYPE3_KC, BOTTOM_ROLLER_TYPE3_TAUI, BOTTOM_ROLLER_TAUD, BOTTOM_ROLLER_INTERVAL);
PIDAaronBerk pidTopThrower(TOP_ROLLER_TYPE3_KC, TOP_ROLLER_TYPE3_TAUI, TOP_ROLLER_TAUD, TOP_ROLLER_INTERVAL);

servoKRAI magazine(SERVO_PIN, SERVO_DEFAULT_ANGLE);
DigitalIn reloaderLimit(MAGAZINE_CHECKER_PIN);

// Shooter
Shooter shooter(&angleMotor, &topRoller, &bottomRoller, &ringPusher, &encoderAngleMotor, &encoderTopThrower, &encoderBottomThrower, &pidTopThrower, &pidBottomThrower);
uint32_t rollerLastToggle = 0;
uint32_t lastShoot = 0;
uint32_t lastReload = 0;
bool autoShakeAble = false;
bool autoShakeReady = false;
bool autoShakeMode = true;
bool isAfterShoot = false;

// Reloader
Reloader magazineReloader(&magazine, &reloaderLimit);

// INA219

/* Throwing Mode Variables */
ThrowingMode throwingModePoleType1Close(SPEED_POLE_TYPE_1_CLOSE_TOP, SPEED_POLE_TYPE_1_CLOSE_BOTTOM);
ThrowingMode throwingModePoleType2Close(SPEED_POLE_TYPE_2_CLOSE_TOP, SPEED_POLE_TYPE_2_CLOSE_BOTTOM);
ThrowingMode throwingModePoleType3(SPEED_POLE_TYPE_3_RED_TOP, SPEED_POLE_TYPE_3_RED_BOTTOM);
ThrowingMode throwingModePoleType1Far(SPEED_POLE_TYPE_1_FAR_TOP, SPEED_POLE_TYPE_1_FAR_BOTTOM);
ThrowingMode throwingModePoleType2Far(SPEED_POLE_TYPE_2_FAR_TOP, SPEED_POLE_TYPE_2_FAR_BOTTOM); // TBD
ThrowingMode throwingModePoleType2Miring(SPEED_POLE_TYPE_2_MIRING_TOP, SPEED_POLE_TYPE_2_MIRING_BOTTOM); // TBD
ThrowingMode throwingModePoleType3Miring(SPEED_POLE_TYPE_3_RED_MIRING_TOP, SPEED_POLE_TYPE_3_RED_MIRING_BOTTOM); // TBD

// Variabel Inisialisasi
float targetSpeedTop = 0;
float targetSpeedBottom = 0;
float targetReloader = 0;
float targetAngleMain = SERVO_DEFAULT_ANGLE;
uint32_t last_moved = us_ticker_read();

// Variabel Kecepatan Roller
bool hasChosenMode = false;

// variable safety reload
bool isReloadSafe = false;

// Time Sampling

/*****************************************************************************/
/*                                THREAD VARIABLES                           */
/*****************************************************************************/

Thread threadBase;
Thread threadStick;
Thread threadShooter;
Thread threadReloader;

/* LED */
// DigitalOut led(LED1);

// Switch Constant
DigitalIn switchConstant(PB_6);
// high merah, low biru

#endif
