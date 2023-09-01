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
float ki_LM = 0.001f;
float kd_LM = 0.00009f;
pidLo pidLeftMotor(kp_LM, ki_LM, kd_LM, samplingPID/1000000.0f, MAXOUT, VFF, RPF, MAXIN);

// 0.027 0.03 0.00008
float kp_ANG = 0.027f;
float ki_ANG = 0.03f;
float kd_ANG = 0.00008f;
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
bool RESET_ANG_MOTOR = true;

// INITIALIZE SERIAL USING PRINTF 
static BufferedSerial serial_port(PA_9, PA_10, 115200);
FileHandle *mbed::mbed_override_console(int fd){
    return &serial_port;
}
