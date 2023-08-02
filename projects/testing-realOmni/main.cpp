// main.cpp
#include "mbed.h"
#include "../../KRAI_Library/Motor/Motor.h"
#include "../../KRAI_Library/encoderKRAI/encoderKRAI.h"
#include "../../KRAI_Library/pidLo/pidLo.h" // Include the pidLo library
#include "../../KRAI_Library/Pinout/F407VET6_2023.h"

//FL motor 4 enc_2_3
//FR motor 3 enc_2_4
//BL motor 2 enc_2_2
//BR motor 1 enc_1_1

// PIN Encoder
#define FL_CHA F407VET6_ENCODER_2_3_A
#define FL_CHB F407VET6_ENCODER_2_3_B

#define FR_CHA F407VET6_ENCODER_2_4_A
#define FR_CHB F407VET6_ENCODER_2_4_B

#define BL_CHA F407VET6_ENCODER_2_2_A
#define BL_CHB F407VET6_ENCODER_2_2_B

#define BR_CHA F407VET6_ENCODER_1_1_A
#define BR_CHB F407VET6_ENCODER_1_1_B
#define PPR 538

// PIN Motor
#define FL_PWM F407VET6_PWM_MOTOR_4
#define FL_FOR F407VET6_FOR_MOTOR_4
#define FL_REV F407VET6_REV_MOTOR_4

#define FR_PWM F407VET6_PWM_MOTOR_3
#define FR_FOR F407VET6_FOR_MOTOR_3
#define FR_REV F407VET6_REV_MOTOR_3

#define BL_PWM F407VET6_PWM_MOTOR_2
#define BL_FOR F407VET6_FOR_MOTOR_2
#define BL_REV F407VET6_REV_MOTOR_2

#define BR_PWM F407VET6_PWM_MOTOR_1
#define BR_FOR F407VET6_FOR_MOTOR_1
#define BR_REV F407VET6_REV_MOTOR_1

// Define paramaeter PID
float Kp = 0.03;
float Ki = 0.8;
float Kd = 0.001;
float setpoint = 130; // SPEED TARGET (RPM) || MAX (2 Cell) => 150 rpm
float Ts = 50 * 0.001; // Sampling time (s)

// Define Object
encoderKRAI FL_enc(FL_CHA, FL_CHB, PPR, Encoding::X4_ENCODING);
encoderKRAI FR_enc(FR_CHA, FR_CHB, PPR, Encoding::X4_ENCODING);
encoderKRAI BL_enc(BL_CHA, BL_CHB, PPR, Encoding::X4_ENCODING);
encoderKRAI BR_enc(BR_CHA, BR_CHB, PPR, Encoding::X4_ENCODING);

Motor FL_motor(FL_PWM, FL_FOR, FL_REV);
Motor FR_motor(FR_PWM, FR_FOR, FR_REV);
Motor BL_motor(BL_PWM, BL_FOR, BL_REV);
Motor BR_motor(BR_PWM, BR_FOR, BR_REV);

#define MAXOUT 1
#define VFF 0
#define RPF 1000
#define MAXIN 1000
pidLo pid(Kp, Ki, Kd, Ts, MAXOUT, VFF, RPF, MAXIN);

float FL_speedRPM, FL_speedPulse;
float FR_speedRPM, FR_speedPulse;
float BL_speedRPM, BL_speedPulse;
float BR_speedRPM, BR_speedPulse;

uint32_t millis_ms(){
    return us_ticker_read()/1000;
}
uint32_t now = millis_ms();
uint32_t SERIAL_PURPOSE_NOW = millis_ms();

int32_t FL_tmpPulse = FL_enc.getPulses();
int32_t FR_tmpPulse = FR_enc.getPulses();
int32_t BL_tmpPulse = BL_enc.getPulses();
int32_t BR_tmpPulse = BR_enc.getPulses();

float PID_error, output;
float motor_default_speed=0.5;

// INITIALIZE SERIAL USING PRINTF 
static BufferedSerial serial_port(PA_9, PA_10, 115200);
FileHandle *mbed::mbed_override_console(int fd){
    return &serial_port;
}

int main()
{
    while (true)
    {
        // Check for incoming serial data
        if (serial_port.readable())
        {
            // Parse the received string to extract KP, KI, KD values
            //scanf("%f %f %f %f", &Kp, &Ki, &Kd, &setpoint);
            // Print the updated values
            //printf("Updated PID values: Kp=%.2f, Ki=%.2f, Kd=%.2f, goal=%.2f\n", Kp, Ki, Kd, setpoint);
            // Set the updated PID values
            //pid.setTunings(Kp, Ki, Kd);

            scanf("%f", &motor_default_speed);
        }

        // Cek Speed berkala setiap Ts detik
        if (millis_ms() - now > Ts * 1000)
        {
            FL_speedPulse = ((float)(FL_enc.getPulses() - FL_tmpPulse) / Ts) * (60.0); //
            FL_tmpPulse = FL_enc.getPulses();
            FL_speedRPM = FL_speedPulse / PPR; // pulse/s / Pulse/rotation = rotation/s

            FR_speedPulse = ((float)(FR_enc.getPulses() - FR_tmpPulse) / Ts) * (60.0); //
            FR_tmpPulse = FR_enc.getPulses();
            FR_speedRPM = FR_speedPulse / PPR; // pulse/s / Pulse/rotation = rotation/s

            BL_speedPulse = ((float)(BL_enc.getPulses() - BL_tmpPulse) / Ts) * (60.0); //
            BL_tmpPulse = BL_enc.getPulses();
            BL_speedRPM = BL_speedPulse / PPR; // pulse/s / Pulse/rotation = rotation/s

            BR_speedPulse = ((float)(BR_enc.getPulses() - BR_tmpPulse) / Ts) * (60.0); //
            BR_tmpPulse = BR_enc.getPulses();
            BR_speedRPM = BR_speedPulse / PPR; // pulse/s / Pulse/rotation = rotation/s
            // Hitung ulang output yang dibutuhkan motor
            // Hitung berdasarkan delta state_sekarang dan state_target
            // PID_error = setpoint - speedRPM;
            // output = pid.createpwm(setpoint, speedRPM, 1.0); // Call the
            //createpwm() function from pidLo to get the output PWM value

            // SET MOTOR SPEED
            FL_motor.speed(motor_default_speed);
            FR_motor.speed(motor_default_speed);
            BL_motor.speed(motor_default_speed);
            BR_motor.speed(motor_default_speed);

            // Coba
            // printf("%f ", output);
            printf("FL:%.2f FR:%.2f BL:%.2f BR: %.2f\n", FL_speedRPM, FR_speedRPM, BL_speedRPM, BR_speedRPM);
            // printf("%f, %f, %ld\n", speedRPM, output ,millis_ms()-SERIAL_PURPOSE_NOW);

            now = millis_ms();
        }   
    }
}