#ifndef CONFIGURATIONPIN_H
#define CONFIGURATIONPIN_H

/****************************************/
/********** DEFINE PINLIST HERE *********/
/****************************************/

// #include "../KRAI_Library/Pinout/F446RE_MASTER_2022.h"
#include "../KRAI_Library/Pinout/F407VET6_2023.h"

//Motor
// BR :D
// FL :E
// FR :A
// BL :B

// Kanan: R = Forward, L = Reverse
// Kiri: R = Reverse, L = Forward

#define motorBR_FOR F407VET6_FOR_MOTOR_4
#define motorBR_REV F407VET6_REV_MOTOR_4
#define motorBR_PWM F407VET6_PWM_MOTOR_4

#define motorFL_FOR F407VET6_FOR_MOTOR_5
#define motorFL_REV F407VET6_REV_MOTOR_5
#define motorFL_PWM F407VET6_PWM_MOTOR_5

#define motorFR_FOR F407VET6_FOR_MOTOR_8
#define motorFR_REV F407VET6_REV_MOTOR_8
#define motorFR_PWM F407VET6_PWM_MOTOR_8

#define motorBL_FOR F407VET6_FOR_MOTOR_1
#define motorBL_REV F407VET6_REV_MOTOR_1
#define motorBL_PWM F407VET6_PWM_MOTOR_1

//Encoder Internal
// FR: C
// BR: B
// BL: A
// FL: D

// Kanan: A dan B sesuai pinout
// Kiri: A dan B dibalik
#define ENC_INTFL_CHA F407VET6_ENCODER_2_2_A
#define ENC_INTFL_CHB F407VET6_ENCODER_2_2_B

#define ENC_INTFR_CHA F407VET6_ENCODER_2_4_A
#define ENC_INTFR_CHB F407VET6_ENCODER_2_4_B

#define ENC_INTBR_CHA F407VET6_ENCODER_1_4_A
#define ENC_INTBR_CHB F407VET6_ENCODER_1_4_B
    
#define ENC_INTBL_CHA F407VET6_ENCODER_1_2_A
#define ENC_INTBL_CHB F407VET6_ENCODER_1_2_B

//UART JOYSTICK
#define UART_RX F407VET6_UART_RX_2
#define UART_TX F407VET6_UART_TX_2

//SPI
// Master
// #define SPI_SCK PA_5
// #define SPI_SS PA_4
// #define SPI_MISO PA_6
// #define SPI_MOSI PA_7

// Ultrasonic, catatan:  ultrasonic 2 bisa konflik dengan enc B master (PB5)
// #define ULTRASONIC1_TRIG PB_4
// #define ULTRASONIC1_ECHO PB_5
// #define ULTRASONIC2_TRIG PC_6
// #define ULTRASONIC2_ECHO PC_7

// RESET MEKANISME
// #define PIN_RESET_MEKANISME PA_1

#endif