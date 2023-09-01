#pragma once

// #include "Constants150.h"
// #include "Constants97Cat.h"
#include "Constants94.h"

// Comment by: Fadhil ==========================================================
// File ini sebelumnya bernama Declarations.h
// Constants yang terdapat pada Variables.h dipindah ke Constants.h,
// sedangkan variables pada file Constants.h dipindah ke Variables.h
// ===========================================================================

/* TIME SAMPLINGS */
#define SAMP_BASE_MOTOR_US 5173
#define SAMP_PID_BASE_MOTOR_US 4173
#define SAMP_BASE_MOTOR_ENCODER_US 5000

#define SAMP_IK_US 20000
#define SAMP_UPD_POS_US 12000
#define SAMP_BASE_ODOMETRY_US 12731

#define SAMP_OTOM_US 500000
#define SAMP_PARALLEL_PARK_US 500000

#define SAMP_STICK_US 13000
#define SAMP_STICK_MS 13

/* BASE MOTOR PID */

/** 
 * Konstanta Untuk PID baru, tidak jadi dipakai
*/
// #define BASE_KC 6.0f
// #define BASE_KC_CEPAT 24.0f
// #define BASE_TAUI 0.105f

// #define BASE_FL_KC 6.0f
// #define BASE_FL_KC_STEADY BASE_FL_KC
// #define BASE_FL_KC_CEPAT 24.0f
// #define BASE_FL_TAUI BASE_TAUI
// #define BASE_FL_TAUD 0
// #define BASE_FL_TS (SAMP_PID_BASE_MOTOR_US/1000000.0)

// #define BASE_FR_KC 6.0f
// #define BASE_FR_KC_STEADY BASE_FR_KC
// #define BASE_FR_KC_CEPAT 24.0f
// #define BASE_FR_TAUI BASE_TAUI
// #define BASE_FR_TAUD 0
// #define BASE_FR_TS (SAMP_PID_BASE_MOTOR_US/1000000.0)

// #define BASE_BR_KC 6.0f
// #define BASE_BR_KC_STEADY BASE_BR_KC
// #define BASE_BR_KC_CEPAT 24.0f
// #define BASE_BR_TAUI BASE_TAUI
// #define BASE_BR_TAUD 0
// #define BASE_BR_TS (SAMP_PID_BASE_MOTOR_US/1000000.0)

// #define BASE_BL_KC 6.0f
// #define BASE_BL_KC_STEADY BASE_BL_KC
// #define BASE_BL_KC_CEPAT 24.0f
// #define BASE_BL_TAUI BASE_TAUI
// #define BASE_BL_TAUD 0
// #define BASE_BL_TS (SAMP_PID_BASE_MOTOR_US/1000000.0)

/** 
 * Konstanta Untuk PID lama, dipakai.
 * Konstanta dengan akhiran '_MAX' tidak dipakai, namun diperlukan untuk constructor ControlMotor
*/
#define BASE_FL_KP 0.9f
#define BASE_FL_KP_MAX 1.0f
#define BASE_FL_KI 0
#define BASE_FL_KI_MAX 0
#define BASE_FL_KD 1.0f
#define BASE_FL_KD_MAX 0.0
#define BASE_FL_TS_MS (SAMP_PID_BASE_MOTOR_US/1000.0)

#define BASE_FR_KP 0.9f
#define BASE_FR_KP_MAX 1.0f
#define BASE_FR_KI 0
#define BASE_FR_KI_MAX 0
#define BASE_FR_KD 1.0f
#define BASE_FR_KD_MAX 0.0
#define BASE_FR_TS_MS (SAMP_PID_BASE_MOTOR_US/1000.0)

#define BASE_BR_KP 0.9f
#define BASE_BR_KP_MAX 1.0f
#define BASE_BR_KI 0
#define BASE_BR_KI_MAX 0
#define BASE_BR_KD 1.0f
#define BASE_BR_KD_MAX 0.0
#define BASE_BR_TS_MS (SAMP_PID_BASE_MOTOR_US/1000.0)

#define BASE_BL_KP 0.9f
#define BASE_BL_KP_MAX 1.0
#define BASE_BL_KI 0
#define BASE_BL_KI_MAX 0
#define BASE_BL_KD 1.0f
#define BASE_BL_KD_MAX 0.0
#define BASE_BL_TS_MS (SAMP_PID_BASE_MOTOR_US/1000.0)

/**
 * Konstanta PID lama untuk braking
*/
#define BASE_BL_KP_BRAKE 1.50f
#define BASE_BR_KP_BRAKE 1.60f
#define BASE_FL_KP_BRAKE 1.70f
#define BASE_FR_KP_BRAKE 1.60f

/* BASE MOTOR SMC */
/**
 * Konstanta SMC untuk berbagai situasi
*/
#define BASE_FL_SMC_KP 0.0028f
#define BASE_FR_SMC_KP 0.004f
#define BASE_BR_SMC_KP 0.0028f
#define BASE_BL_SMC_KP 0.0033f

#define BASE_FL_SMC_KP_BOOST 0.00055f
#define BASE_FR_SMC_KP_BOOST 0.0085f
#define BASE_BR_SMC_KP_BOOST 0.0023f
#define BASE_BL_SMC_KP_BOOST 0.0015f

#define BASE_FL_SMC_KP_BRAKE 2.0f
#define BASE_FR_SMC_KP_BRAKE 2.5f
#define BASE_BR_SMC_KP_BRAKE 2.2f
#define BASE_BL_SMC_KP_BRAKE 2.2f

#define BASE_FL_SMC_KP_MAX 0.001f
#define BASE_FR_SMC_KP_MAX 0.001f
#define BASE_BR_SMC_KP_MAX 0.001f
#define BASE_BL_SMC_KP_MAX 0.001f

#define SMC_FL_KSIGMA 10
#define SMC_FR_KSIGMA 15
#define SMC_BR_KSIGMA 10
#define SMC_BL_KSIGMA 12

#define SMC_FL_KSIGMA_BRAKE 50
#define SMC_FR_KSIGMA_BRAKE 50
#define SMC_BR_KSIGMA_BRAKE 50
#define SMC_BL_KSIGMA_BRAKE 50

#define SMC_EPSILON 0.01f
#define SMC_BETA 1
#define SMC_SAMPLING (SAMP_PID_BASE_MOTOR_US/1000000.0)

/* BASE MOTOR CONTROL */
// ga dipake tp jg dibutuhin bwt ControlMotor
#define BASE_MOTOR_V_LIM 1.0f

/* BASE MOTOR VARIABLE */
#define ENC_BASE_MOTOR_PPR 537.6f

#define TRANSLATION_BASE_SPEED 0.45f
#define ROTATION_BASE_SPEED (1.05 * PI / 6)
#define TRANSLATION_BOOST_MULTIPLIER 2.15f
#define ROTATION_BOOST_MULTIPLIER 1.8f
#define TRANSLATION_HINDER_MULTIPLIER 0.4f
#define ROTATION_HINDER_MULTIPLIER 0.4f

/* BASE MOTOR INPUT LIMIT */
#define MAX_FL_INPUT_LIMIT 3.3 // 2.6
#define MAX_FR_INPUT_LIMIT 3.3 // 2.7
#define MAX_BR_INPUT_LIMIT 3.3 // 2.2
#define MAX_BL_INPUT_LIMIT 3.3 // 2.8

/* CONTROL4OMNI VARIABLES */
/* Kalau mau ubah, perlu ubah sampai library-nya, jangan diapa-apain */
#ifndef PI
#define PI 3.1415926535897932384626433832795f
#endif

#define ERROR_THRESHOLD 5

#define ENC_MOTOR_SAMP_US_DEF SAMP_BASE_MOTOR_ENCODER_US
#define ENC_MOTOR_PULSE ENC_BASE_MOTOR_PPR

#define WHEEL_RAD 0.075

#define S_TO_US 1000000
#define MS_TO_US 1000

#define MAX_ACCEL_Y 8
#define MAX_ACCEL_X 8
#define MAX_ACCEL_W (20 * PI)

#define R_BASE 0.3341295
// #define R_BASE 0.450

// Servo
#define SERVO_DEFAULT_ANGLE -53.0

#define MAXOUT 1
#define VFF 0
#define RPF 1000
#define MAXIN 1000

// AIMING TARGET
#define TARGETLINE1 0
#define TARGETLINE2 4
#define TARGETLINE3 8
#define TARGETLINE4 12
#define TARGETLINE5 16

#define DELTAAIM 0.05

// RELOADER
#define RELOADER_TOGGLE_DELAY 500000
#define POSITION_DOWN_ANGLE 84.7
#define POSITION_STANDBY_ANGLE SERVO_DEFAULT_ANGLE
#define DURATION_FOR_SHAKE_UP 150 * 1000
#define DURATION_OF_GOING_UP_OR_DOWN 1000 * 1000
#define DURATION_FOR_GOING_DOWN 750 * 1000
#define DURATION_FOR_GOING_DOWN_DELAY_MS 5
#define SHAKE_DEGREE 20
#define AUTO_SHAKE_DELAY 600000
#define AUTO_SHAKE_DURATION 800000