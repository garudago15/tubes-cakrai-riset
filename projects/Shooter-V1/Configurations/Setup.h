#pragma once

#include "mbed.h"
#include "../../../KRAI_Library/encoderKRAI/encoderKRAI.h"
#include "../../../KRAI_Library/Motor/Motor.h"
#include "../../../KRAI_Library/pidLo/pidLo.h"
#include "../../../KRAI_Library/MovingAverage/MovingAverage.h"
#include "Constants.h"

/* Time Sampling
    1.000.000 mikroSecond = 1 second
    1.000     mikroSecond = 1 miliSecond
*/
uint32_t samplingPID = 10 * 1000; // Mikro Second

// PID (Pernah motor pg45, tanpa beban, berhasil dituning dengan konstanta berikut)
// float kp_LM = 0.00538f;
// float ki_LM = 0.009f;
// float kd_LM = 0.0004f;

// PID DESIGN FAHMI
// float kp_LM = 0.00838f;
// float ki_LM = 0.04f;
// float kd_LM = 0.00038f;

float kp_LM = 0.001f;
float ki_LM = 0.04f;
float kd_LM = 0.00008f;
pidLo pidLeftMotor(kp_LM, ki_LM, kd_LM, samplingPID/1000000.0f, MAXOUT, VFF, RPF, MAXIN);

float kp_ANG = 0.0f;
float ki_ANG = 0.0f;
float kd_ANG = 0.0f;
pidLo pidMotorAngle(kp_ANG, ki_ANG, kd_ANG, samplingPID/1000000.0f, MAXOUT, VFF, RPF, MAXIN);

// Encoder and Motor Shooter
// Motor leftMotor(PWM_LM, FOR_LM, REV_LM);
Motor motorReload(PWM_RLD, FOR_RLD, REV_RLD);
encoderKRAI encMotorReload(CHA_RLD, CHB_RLD, PPR_RLD, Encoding::X4_ENCODING);

Motor leftMotor(PWM_LM, FOR_LM, REV_LM);
encoderKRAI encLeftMotor(CHA_LM, CHB_LM, PPR_LM, Encoding::X4_ENCODING);

Motor motorAngle(PWM_ANG, FOR_ANG, REV_ANG);
encoderKRAI encMotorAngle(CHA_ANG, CHB_ANG, PPR_ANG, Encoding::X4_ENCODING);
InterruptIn LimitSwitch(LIMIT_SWITCH_PIN, PullDown);

// Moving Average
MovingAverage movAvgLM(20);
MovingAverage movAvgAccel(5);
MovingAverage movAvgANG(10);

// Variabel Inisialisasi
float timeLast = us_ticker_read();
float timeLastForAngleShooter = us_ticker_read();
bool RESET_ANG_MOTOR = false;

// INITIALIZE SERIAL USING PRINTF 
static BufferedSerial serial_port(PA_9, PA_10, 115200);
FileHandle *mbed::mbed_override_console(int fd){
    return &serial_port;
}

/* ================================= BASE OMNI WHEEL ===========================*/

/* Base Motor */
Motor baseMotorFL(BASE_MOTOR_FL_PWM, BASE_MOTOR_FL_FOR, BASE_MOTOR_FL_REV);
Motor baseMotorFR(BASE_MOTOR_FR_PWM, BASE_MOTOR_FR_FOR, BASE_MOTOR_FR_REV);
Motor baseMotorBL(BASE_MOTOR_BL_PWM, BASE_MOTOR_BL_FOR, BASE_MOTOR_BL_REV);
Motor baseMotorBR(BASE_MOTOR_BR_PWM, BASE_MOTOR_BR_FOR, BASE_MOTOR_BR_REV);

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

/* Moving Average*/
MovingAverage movAvgFL(15);
MovingAverage movAvgFR(15);
MovingAverage movAvgBL(15);
MovingAverage movAvgBR(15);