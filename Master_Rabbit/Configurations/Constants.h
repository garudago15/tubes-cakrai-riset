#ifndef _DECLARATIONS_H_
#define _DECLARATIONS_H_

// Comment by: Fadhil ==========================================================
// File ini sebelumnya bernama Declarations.h
// Constants yang terdapat pada Variables.h dipindah ke Constants.h, 
// sedangkan variables pada file Constants.h dipindah ke Variables.h
// ===========================================================================

/* SAMPLING */
#define PID_MOTOR_SAMP_US 4173

#define MS_TO_US 1000

#define V_batas 1

/* PID MOTOR */     
#define FL_kp       0.9 * 0.7
#define FL_kp1      1
#define FL_ki       0
#define FL_ki1      0
#define FL_kd       1 * 0.7
#define FL_kd1      0
#define FL_N        0.0
#define FL_TS_ms    7        
#define FL_ka       0.0337 
#define FL_kb       0.0382   

#define FR_kp       0.9 * 0.7
#define FR_kp1      1
#define FR_ki       0
#define FR_ki1      0
#define FR_kd       1 * 0.7
#define FR_kd1      0
#define FR_N        0.0
#define FR_TS_ms    7        
#define FR_ka       0.0337 
#define FR_kb       0.0382 

#define BR_kp       0.9 * 0.7
#define BR_kp1      1
#define BR_ki       0
#define BR_ki1      0
#define BR_kd       1 * 0.7
#define BR_kd1      0
#define BR_N        0.0
#define BR_TS_ms    7        
#define BR_ka       0.0337 
#define BR_kb       0.0382 

#define BL_kp       0.9 * 0.7
#define BL_kp1      1
#define BL_ki       0
#define BL_ki1      0
#define BL_kd       1 * 0.7
#define BL_kd1      0
#define BL_N        0.0
#define BL_TS_ms    7        
#define BL_ka       0.0337 
#define BL_kb       0.038

#define BASE_FL_KC 10
#define BASE_FL_TAUI 0.03
#define BASE_FL_TAUD 0.000001
#define BASE_FL_TS 0.004173

#define BASE_FR_KC 10
#define BASE_FR_TAUI 0.03
#define BASE_FR_TAUD 0.000001
#define BASE_FR_TS 0.004173

#define BASE_BL_KC 10
#define BASE_BL_TAUI 0.03
#define BASE_BL_TAUD 0.000001
#define BASE_BL_TS 0.004173

#define BASE_BR_KC 10
#define BASE_BR_TAUI 0.03
#define BASE_BR_TAUD 0.000001
#define BASE_BR_TS 0.004173

/* SMC */
#define FL_SMBR_kp  0.005
#define FR_SMBR_kp  0.005
#define BR_SMBR_kp  0.005
#define BL_SMBR_kp  0.005
#define FL_SMBR_kp1 0.0015
#define FR_SMBR_kp1 0.0015
#define BR_SMBR_kp1 0.0015
#define BL_SMBR_kp1 0.0015

#define SMC_KSIGMA      1
#define SMC_EPSILON     0.01
#define SMC_BETA        1
#define SMC_SAMPLING    5  

// ControlKRAI
#ifndef PI
#define PI                      3.14159265358979
#endif

#define ULTRASONIC_GAP          5
#define WHEEL_RAD               0.01
#define S_TO_US                 1000000
#define ENC_MOTOR_PULSE         538
#define ENC_MOTOR_SAMP_US_DEF   5000
#define SAMP_BASE_SPEED_US      12731

// BASE SQUARE
#define R_BASE                  0.3341295

/* INVERSE KINEMATICS */
#define MAX_ACCEL_Y 30
#define MAX_ACCEL_X 30
#define MAX_ACCEL_W (5*PI)

/* Main */
#define INIT_SCK                1000000
#define STATE_IGN               0xFF
#define SAMP_UPD_POS_US         12000
#define SAMP_STICK_US           13000
#define MOTOR_SAMP_US           5173
#define SAMP_IK_US_DEF          20000
#define SAMP_PRINT              500000
#define SAMP_OTOM_US            500000
#define SAMP_ESP_US             100000
#define SAMP_PARALLEL_PARK_US   500000
#define SAMP_AUTO_AIM_US        13000

/* Control4Roda */
#define ERROR_THRESHOLD 5

/* BluePill */
#define TOTAL_BLUEPILL_DATA 2

#define MAXIMUM_BUFFER_SIZE 10
#define MAXIMUM_BUFFER2_SIZE 100

/* SPI */
#define SAMP_SPI_DATA 500

#endif
