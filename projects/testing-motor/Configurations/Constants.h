#pragma once

#include "../../KRAI_Library/Pinout/F446RE_MASTER_2022.h"
#include "../../KRAI_Library/Pinout/F407VET6_2023.h"

# define PI 3.1415926535

/*
    LM = Left Motor
    RM = Right Motor
*/

// PIN Encoder
#define CHA_LM F407VET6_ENCODER_2_4_A
#define CHB_LM F407VET6_ENCODER_2_4_B
// #define PPR_LM 538
#define PPR_LM 103.3

// Rotary Encoder
#define CHA_ROT F446RE_MASTER_ENCODER_A_D_A
#define CHB_ROT F446RE_MASTER_ENCODER_A_D_B

// PIN Motor
#define PWM_LM F407VET6_PWM_MOTOR_5
#define FOR_LM F407VET6_FOR_MOTOR_5
#define REV_LM F407VET6_REV_MOTOR_5

// PIN Motor Reload
#define PWM_RLD F407VET6_PWM_MOTOR_8 //reloader
#define FOR_RLD F407VET6_FOR_MOTOR_8
#define REV_RLD F407VET6_REV_MOTOR_8

#define CHA_RLD F407VET6_ENCODER_2_2_A //reloader
#define CHB_RLD F407VET6_ENCODER_2_2_B
#define PPR_RLD 104.4

// Limit Switch Sudut
#define LIMIT_SWITCH_PIN F407VET6_ENCODER_2_1_B

// PIDLO
#define MAXOUT 1
#define VFF 0
#define RPF 1000
#define MAXIN 1000

/* JOYSTICK UART */
#define UART_STICK_TX F407VET6_UART_TX_2
#define UART_STICK_RX F407VET6_UART_RX_2

// Encoder Sudut
#define CHA3 F407VET6_ENCODER_2_3_A //sudut
#define CHB3 F407VET6_ENCODER_2_3_B

// Motor Sudut
#define PWM3 F407VET6_PWM_MOTOR_7 //sudut
#define FOR3 F407VET6_FOR_MOTOR_7
#define REV3 F407VET6_REV_MOTOR_7