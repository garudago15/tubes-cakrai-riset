#ifndef _VARIABLES_H_
#define _VARIABLES_H_

#include "ConfigurationPin.h"
#include "Constants.h"
#include "../../KRAI_Library/Motor/Motor.h"
#include "../../KRAI_Library/pidLo/pidLo.h"
#include "../../KRAI_Library/SMC_KRAI/SMC_KRAI.h"
#include "../../KRAI_Library/ControlMotor/ControlMotor.h"
#include "../../KRAI_Library/encoderHAL/encoderHAL.h"
#include "../../KRAI_Library/encoderKRAI/encoderKRAI.h"
#include "../../KRAI_Library/ultrasonicKRAI/ultrasonicKRAI.h"
#include "../../KRAI_Library/Control4Roda/ControlAutomatic4Mechanum.h"
#include "../../KRAI_Library/StanleyPursuit/StanleyPursuit.h"
#include "SPI.h"
#include "../../KRAI_Library/encoderHAL/EncoderMspInitF4.h"
#include "../../KRAI_Library/odom3enc/Coordinate.h"
#include "../../KRAI_Library/odom3enc/odom3enc.h"
#include "../../KRAI_Library/odom2enc/odom2enc.h"
#include "../../KRAI_Library/JoystickPS3/JoystickPS3.h"

// Comment by: Fadhil ==========================================================
// Variables yang terdapat pada Declarations.h dipindah ke Variables.h, 
// sedangkan konstanta Makro pada file Variables.h dipindah ke Constants.h
// ===========================================================================

char ALPHABET[26] = {'A','B','C','D','E','F','G','H','I','J','K','L','M','N','O','P','Q','R','S','T','U','V','W','X','Y','Z'};

std::chrono::microseconds ENC_MOTOR_SAMP_US(20ms);
std::chrono::microseconds SAMP_IK_US(15ms);

string data[TOTAL_BLUEPILL_DATA];

/* TIME SAMPLING */
uint32_t samplingPID = 0;
uint32_t samplingOdom = 0;
uint32_t samplingStick = 0;
uint32_t samplingMotor = 0;
uint32_t samplingUpdPos = 0;
uint32_t samplingEncoder = 0;
uint32_t samplingIK = 0;
uint32_t samplingPrint = 0;
uint32_t samplingOtomatis = 0;
uint32_t samplingOtomatisESP = 0;
uint32_t samplingParallelPark = 0;
uint32_t samplingAutoAim = 0;

/* SPI */
uint32_t data_sampling_spi = 0;
uint8_t data_send_spi = 0;
uint8_t resp = 0;
int send_spi_inside;
uint8_t state_send;

/* MOTOR */
// pwm, FOR, rev
Motor FL_motor           (motorFL_PWM, motorFL_FOR, motorFL_REV);
Motor FR_motor           (motorFR_PWM, motorFR_FOR, motorFR_REV);
Motor BL_motor           (motorBL_PWM, motorBL_FOR, motorBL_REV);
Motor BR_motor           (motorBR_PWM, motorBR_FOR, motorBR_REV);

/* PID Vx, Vy, W */
pidLo vxPid(0.05, 0.01, 0, SAMP_IK_US_DEF, 10, 1, 1000, 100);
pidLo vyPid(0.05 , 0.01, 0, SAMP_IK_US_DEF, 10, 1, 1000, 100);
pidLo wPid(0.025, 0.0025, 0, SAMP_IK_US_DEF, 10, 1, 1000, 100);

/* PID untuk motor */
pidLo FL_pid_motor(FL_kp, FL_ki, FL_kd, FL_TS_ms, 1, 0, 1000, 100);
pidLo FR_pid_motor(FR_kp, FR_ki, FR_kd, FR_TS_ms, 1, 0, 1000, 100);
pidLo BR_pid_motor(BR_kp, BR_ki, BR_kd, BR_TS_ms, 1, 0, 1000, 100);
pidLo BL_pid_motor(BL_kp, BL_ki, BL_kd, BL_TS_ms, 1, 0, 1000, 100);

PIDAaronBerk pidAaronBerkMotorFL(BASE_FL_KC, BASE_FL_TAUI, BASE_FL_TAUD, BASE_FL_TS);
PIDAaronBerk pidAaronBerkMotorFR(BASE_FR_KC, BASE_FR_TAUI, BASE_FR_TAUD, BASE_FR_TS);
PIDAaronBerk pidAaronBerkMotorBR(BASE_BR_KC, BASE_BR_TAUI, BASE_BR_TAUD, BASE_BR_TS);
PIDAaronBerk pidAaronBerkMotorBL(BASE_BL_KC, BASE_BL_TAUI, BASE_BL_TAUD, BASE_BL_TS);

/* SMC untuk motor */
SMC FL_smc_motor(FL_SMBR_kp, SMC_KSIGMA, SMC_EPSILON, SMC_BETA, (float)PID_MOTOR_SAMP_US/MS_TO_US, SMC::KECEPATAN);
SMC FR_smc_motor(FR_SMBR_kp, SMC_KSIGMA, SMC_EPSILON, SMC_BETA, (float)PID_MOTOR_SAMP_US/MS_TO_US, SMC::KECEPATAN);
SMC BR_smc_motor(BR_SMBR_kp, SMC_KSIGMA, SMC_EPSILON, SMC_BETA, (float)PID_MOTOR_SAMP_US/MS_TO_US, SMC::KECEPATAN);
SMC BL_smc_motor(BL_SMBR_kp, SMC_KSIGMA, SMC_EPSILON, SMC_BETA, (float)PID_MOTOR_SAMP_US/MS_TO_US, SMC::KECEPATAN);

/*Control Motor */
// ControlMotor control_FL_motor(&FL_pid_motor, &FL_smc_motor, V_batas, FL_kp, FL_kp1, FL_kd, FL_kd1, FL_SMBR_kp, FL_SMBR_kp1);
// ControlMotor control_FR_motor(&FR_pid_motor, &FR_smc_motor, V_batas, FR_kp, FR_kp1, FR_kd, FR_kd1, FR_SMBR_kp, FR_SMBR_kp1);
// ControlMotor control_BR_motor(&BR_pid_motor, &BR_smc_motor, V_batas, BR_kp, BR_kp1, BR_kd, BR_kd1, BR_SMBR_kp, BR_SMBR_kp1);
// ControlMotor control_BL_motor(&BL_pid_motor, &BL_smc_motor, V_batas, BL_kp, BL_kp1, BL_kd, BL_kd1, BL_SMBR_kp, BL_SMBR_kp1);

ControlMotor control_FL_motor(&pidAaronBerkMotorFL, &FL_smc_motor, V_batas, FL_kp, FL_kp1, FL_kd, FL_kd1, FL_SMBR_kp, FL_SMBR_kp1);
ControlMotor control_FR_motor(&pidAaronBerkMotorFR, &FR_smc_motor, V_batas, FR_kp, FR_kp1, FR_kd, FR_kd1, FR_SMBR_kp, FR_SMBR_kp1);
ControlMotor control_BR_motor(&pidAaronBerkMotorBR, &BR_smc_motor, V_batas, BR_kp, BR_kp1, BR_kd, BR_kd1, BR_SMBR_kp, BR_SMBR_kp1);
ControlMotor control_BL_motor(&pidAaronBerkMotorBL, &BL_smc_motor, V_batas, BL_kp, BL_kp1, BL_kd, BL_kd1, BL_SMBR_kp, BL_SMBR_kp1);


/* ENCODER HAL (ODOMETRI) */
encoderHAL encL(TIM3);
// encoderHAL encR(TIM4);
encoderHAL encAux(TIM8);

/* ENCODER KRAI (BASE MOTOR) */
encoderKRAI encFL        (ENC_INTFL_CHA, ENC_INTFL_CHB, 538, X4_ENCODING);
encoderKRAI encFR        (ENC_INTFR_CHA, ENC_INTFR_CHB, 538, X4_ENCODING);
encoderKRAI encBR        (ENC_INTBR_CHA, ENC_INTBR_CHB, 538, X4_ENCODING);
encoderKRAI encBL        (ENC_INTBL_CHA, ENC_INTBL_CHB, 538, X4_ENCODING);

/* JOYSTICK */
JoystickPS3 stick(UART_TX, UART_RX);

/* ODOMETRI */
odom2enc odom(&encAux, &encL);

/* TRAJECTORY FOLLOWING */
StanleyPursuit line;
pidLo pid(0.08, 0.05, 0.5, 0.5 , 0.5, 0, 1000, 100);
pidLo pid2(0.03, 0.005, 0.2, 0.5 , 0.5, 0, 1000, 100);

/* ControlKRAI */
ControlAutomatic4Mechanum controlRabbit(&FL_motor, &FR_motor, &BR_motor, &BL_motor, &encFL, &encFR, &encBR, &encBL, &control_FL_motor, &control_FR_motor, &control_BR_motor, &control_BL_motor, &odom, &vxPid, &vyPid, &wPid, &line, &pid, &pid2);



// static BufferedSerial serial_port2(BP_TX, BP_RX, 115200);

// /* BLUEPHILL */
// int timer = 0;
// char buf[MAXIMUM_BUFFER_SIZE] = {0};
// char buf2[MAXIMUM_BUFFER2_SIZE] = {0};

// string datas[TOTAL_BLUEPILL_DATA];

// int data_get = 0;
// int data_index = 0;

// /* ULTRASONIC */
// float ultR = 0;
// float ultL = 0;
// ultrasonicKRAI ultrasonicR(ULTRASONIC1_TRIG, ULTRASONIC1_ECHO, &ultR);
// ultrasonicKRAI ultrasonicL(ULTRASONIC2_TRIG, ULTRASONIC2_ECHO, &ultL);

/* LED */
DigitalOut led(LED1);

/* RESET MEKANISME */
// DigitalOut resetMekanisme(PIN_RESET_MEKANISME);

#endif