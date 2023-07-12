#ifndef _CONSTANTS_H_
#define _CONSTANTS_H_


#define TARGET_STM32F4
#define TIME_SAMPLING 50 // Time Sampling PID dalam us
//note: kalau jelek, mungkin bisa coba 1.5 us sesuai library omniWheel
#define S_TO_US 1000000
#define PI 3.14159265359


/* PS3 JOYSTICK CONSTANTS */
#define PS3_RX PC_10
#define PS3_TX PC_11


/* SHOOTER CONSTANTS */
#define FL_PWM PA_0
#define FL_FWD PA_0
#define FL_REV PA_0

#define FR_PWM PA_0
#define FR_FWD PA_0
#define FR_REV PA_0

#define BL_PWM PA_0
#define BL_FWD PA_0
#define BL_REV PA_0

#define BR_PWM PA_0
#define BR_FWD PA_0
#define BR_REV PA_0

#define FLY_PWM PA_0
#define FLY_FWD PA_0
#define FLY_REV PA_0

#define ANG_PWM PA_0
#define ANG_FWD PA_0
#define ANG_REV PA_0

#define RLD_PWM PA_0
#define RLD_FWD PA_0
#define RLD_REV PA_0

#define FLY_CHA PA_0
#define FLY_CHB PA_0

#define ANG_CHA PA_0
#define ANG_CHB PA_0

#define RLD_CHA PA_0
#define RLD_CHB PA_0

#define FLY_PPR 105 // PPR Motor Flywheel 1 (PERLU TESTING)
#define ANG_PPR 105 // PPR Motor Pengatur sudut shooter bola
#define RLD_PPR 105 // PPR Motor Reloader
#define RLD_ANGLE 72 // Sudut yang diperlukan untuk 1 bola keluar dari reloader menuju flywheel

#define FLY1_RAD 0.16 // (m) Radius flywheel 1 
#define FLY2_RAD 0.16 // (m) Radius flywheel 2
#define FLY1_MASS 0.5 // (kg) Massa flywheel 1
#define FLY2_MASS 0.5 // (kg) Massa flywheel 2


/* OMNIWHEEL CONSTANTS */
#define FL_PWM PA_0 //asumsi
#define FL_FWD PA_0 //asumsi
#define FL_REV PA_0 //asumsi

#define FR_PWM PA_0 //asumsi
#define FR_FWD PA_0 //asumsi
#define FR_REV PA_0 //asumsi

#define BR_PWM PA_0 //asumsi
#define BR_FWD PA_0 //asumsi
#define BR_REV PA_0 //asumsi

#define BL_PWM PA_0 //asumsi
#define BL_FWD PA_0 //asumsi
#define BL_REV PA_0 //asumsi

#define FL_CHA PA_0 //asumsi
#define FL_CHB PA_0 //asumsi

#define FR_CHA PA_0 //asumsi
#define FR_CHB PA_0 //asumsi

#define BR_CHA PA_0 //asumsi
#define BR_CHB PA_0 //asumsi

#define BL_CHA PA_0 //asumsi
#define BL_CHB PA_0 //asumsi

#define KP_WHEEL_PID 0 //asumsi
#define KI_WHEEL_PID 0 //asumsi

#define KP_WHEEL_SMC 0 //asumsi
#define KS_WHEEL_SMC 0 //asumsi
#define EPS_WHEEL_SMC 0.001 //asumsi

#define KP_WHEEL_PID_MIN 0 // asumsi
#define KP_WHEEL_PID_MAX 0 // asumsi
#define KD_WHEEL_PID_MIN 0 // asumsi
#define KD_WHEEL_PID_MAX 0 // asumsi
#define KP_WHEEL_SMC_MIN 0 // asumsi
#define KP_WHEEL_SMC_MAX 0 // asumsi

#define encL_TIM TIM2 // pin... (CH1) dan pin ... (CH2) //asumsi
#define encR_TIM TIM3 // pin... (CH1) dan pin ... (CH2) //asumsi
#define encAux_TIM TIM4 // pin... (CH1) dan pin ... (CH2) //asumsi

#define ERROR_THRESHOLD 0.01 //tolerable error utk automatic positioning //default 0
#define MAX_ACCEL_X 1 //dalam m/s^2 //asumsi
#define MAX_ACCEL_Y 1 //dalam m/s^2 //asumsi
#define MAX_ROBOT_SPEED 2 //dalam m/s //asumsi
#define MAX_ROBOT_SPIN 5.6 // bisa didapat dari MAX_WHEEL_SPEED / R_BASE //asumsi
#define MAX_WHEEL_SPEED 2.8 // bisa didapat dari MAX_ROBOT_SPEED / cos45 (dalam m/s) //asumsi

#define KP_ROBOT_PID_v 0 // asumsi
#define KI_ROBOT_PID_v 0 // asumsi
#define KP_ROBOT_PID_w 0 // asumsi
#define KI_ROBOT_PID_w 0 // asumsi

#define PPRX 2060 // 1000 // karena x4 jadi ppr * 4 = 1000 //dr library
#define PPRY 2060 // 1000 // karena x4 jadi ppr * 4 = 1000 //dr library
#define D_RODA 0.058 // Diameter roda encoder (dalam m) //dr libary
#define R 0.51662// R: jarak pusat ke encL&encR (dalam m) //dr library
#define K 0.108// K: jarak pusat ke encAux (dalam m) //dr library
#define WHEEL_RAD 0.075 //radius omniwheel //asumsi
#define WHEEL_PPR 105 //PPR omniwheel //asumsi
#define R_BASE 0.5 //jarak omniwheel ke pusat //asumsi


#endif