#ifndef CONFIGURATIONPIN_H
#define CONFIGURATIONPIN_H

#include "../../KRAI_Library/Pinout/F407VET6_2023.h"

/*                                      *\

                PIN LIST

*/

/* BASE MOTOR */

/* Current Mapping
 *
 * FR : 3
 * BR : 2
 * BL : 1
 * FL : 4
*/
#define BASE_MOTOR_BR_FOR F407VET6_REV_MOTOR_1
#define BASE_MOTOR_BR_REV F407VET6_FOR_MOTOR_1
#define BASE_MOTOR_BR_PWM F407VET6_PWM_MOTOR_1

#define BASE_MOTOR_FR_FOR F407VET6_REV_MOTOR_6
#define BASE_MOTOR_FR_REV F407VET6_FOR_MOTOR_6
#define BASE_MOTOR_FR_PWM F407VET6_PWM_MOTOR_6

#define BASE_MOTOR_BL_FOR F407VET6_REV_MOTOR_2
#define BASE_MOTOR_BL_REV F407VET6_FOR_MOTOR_2
#define BASE_MOTOR_BL_PWM F407VET6_PWM_MOTOR_2

#define BASE_MOTOR_FL_FOR F407VET6_REV_MOTOR_4
#define BASE_MOTOR_FL_REV F407VET6_FOR_MOTOR_4
#define BASE_MOTOR_FL_PWM F407VET6_PWM_MOTOR_4

#define MOTOR_SHOOTER_FOR F407VET6_FOR_MOTOR_1
#define MOTOR_SHOOTER_REV F407VET6_REV_MOTOR_1
#define MOTOR_SHOOTER_PWM F407VET6_PWM_MOTOR_1

#define MOTOR_RELOADER_FOR F407VET6_FOR_MOTOR_5
#define MOTOR_RELOADER_REV F407VET6_REV_MOTOR_5
#define MOTOR_RELOADER_PWM F407VET6_PWM_MOTOR_5

#define MOTOR_ANGLE_FOR F407VET6_FOR_MOTOR_7
#define MOTOR_ANGLE_REV F407VET6_REV_MOTOR_7
#define MOTOR_ANGLE_PWM F407VET6_PWM_MOTOR_7

//NOTE: pin motor_3 bermasalah

/* BASE MOTOR ENCODER */
/* Current Mapping
 *
 * BL : 1-3
 * BR : 1-4
 * FR : 1-2
 * FL : 1-1
*/
#define ENCODER_BASE_MOTOR_BR_CHA F407VET6_ENCODER_1_4_B
#define ENCODER_BASE_MOTOR_BR_CHB F407VET6_ENCODER_1_4_A

#define ENCODER_BASE_MOTOR_BL_CHA F407VET6_ENCODER_1_3_B
#define ENCODER_BASE_MOTOR_BL_CHB F407VET6_ENCODER_1_3_A

#define ENCODER_BASE_MOTOR_FR_CHA F407VET6_ENCODER_1_2_B // Di Nigel ini _3_A
#define ENCODER_BASE_MOTOR_FR_CHB F407VET6_ENCODER_1_2_A // Di Nigel ini _3_B

#define ENCODER_BASE_MOTOR_FL_CHA F407VET6_ENCODER_1_1_B // Di Nigel ini _4_B
#define ENCODER_BASE_MOTOR_FL_CHB F407VET6_ENCODER_1_1_A // Di Nigel ini _4_A

#define ENCODER_SHOOTER_CHA F407VET6_ENCODER_2_4_A
#define ENCODER_SHOOTER_CHB F407VET6_ENCODER_2_4_B

#define ENCODER_RELOADER_CHA F407VET6_ENCODER_2_2_A
#define ENCODER_RELOADER_CHB F407VET6_ENCODER_2_2_B

#define ENCODER_ANGLE_CHA F407VET6_ENCODER_2_3_A
#define ENCODER_ANGLE_CHB F407VET6_ENCODER_2_3_B

/* ODOMETRY */
// #define ENCODER_L_TIM   TIM1    // F407VET6_ENCODER_2_1
#define ENCODER_L_CHA   F407VET6_ENCODER_1_3_A
#define ENCODER_L_CHB   F407VET6_ENCODER_1_3_B

// #define ENCODER_AUX_TIM TIM3    // F407VET6_ENCODER_2_3
#define ENCODER_AUX_CHA F407VET6_ENCODER_1_4_A
#define ENCODER_AUX_CHB F407VET6_ENCODER_1_4_B

/* JOYSTICK UART */
#define UART_STICK_TX F407VET6_UART_TX_2
#define UART_STICK_RX F407VET6_UART_RX_2

/* SERIAL PORT UART */
#define UART_SERIAL_TX F407VET6_UART_TX_1
#define UART_SERIAL_RX F407VET6_UART_RX_1

#endif
