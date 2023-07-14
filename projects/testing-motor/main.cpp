/* ori
#include "mbed.h"
#include "../../KRAI_Library/Motor/Motor.h"
#include "../../KRAI_Library/encoderKRAI/encoderKRAI.h"
#include "../../KRAI_Library/pinout/F407VET6_2023.h"

// PIN Encoder
#define CH1A F407VET6_ENCODER_1_1_A
#define CH1B F407VET6_ENCODER_1_1_B
#define PPR 538

#define CH2A F407VET6_ENCODER_1_2_A
#define CH2B F407VET6_ENCODER_1_2_B

#define CH3A F407VET6_ENCODER_1_3_A
#define CH3B F407VET6_ENCODER_1_3_B

#define CH4A F407VET6_ENCODER_1_4_A
#define CH4B F407VET6_ENCODER_1_4_B

// PIN Motor
#define PWM F407VET6_PWM_MOTOR_2
#define FOR F407VET6_FOR_MOTOR_2
#define REV F407VET6_REV_MOTOR_2

//variables
float speedPulse1,speedRPM1;
int tmpPulse1;
float speedPulse2,speedRPM2;
int tmpPulse2;
float speedPulse3,speedRPM3;
int tmpPulse3;
float speedPulse4,speedRPM4;
int tmpPulse4;
uint32_t Ts=100000; //4173

// Define Object
encoderKRAI enc1(CH1A, CH1B, PPR, Encoding::X4_ENCODING);
encoderKRAI enc2(CH2A, CH2B, PPR, Encoding::X4_ENCODING);
encoderKRAI enc3(CH3A, CH3B, PPR, Encoding::X4_ENCODING);
encoderKRAI enc4(CH4A, CH4B, PPR, Encoding::X4_ENCODING);
Motor motor(PWM, FOR, REV);

// INITIALIZE SERIAL USING PRINTF 
static BufferedSerial serial_port(PA_9, PA_10, 115200);
FileHandle *mbed::mbed_override_console(int fd){
    return &serial_port;
}

int main()
{
    uint32_t now = us_ticker_read();
    while (true)
    {
    // Cek Speed berkala setiap Ts detik
    if (us_ticker_read() - now > Ts)
    {
        speedPulse1 = ((float)(enc1.getPulses() - tmpPulse1) / (float)Ts) * (60.0);
        //Calculate the speed in pulses per second and convert to RPM
        tmpPulse1 = enc1.getPulses();
        // KALKULASI SPEED AKHIR
        speedRPM1 = speedPulse1 / (float)PPR; // pulse/s / Pulse/rotation = rotation/s

        speedPulse2 = ((float)(enc2.getPulses() - tmpPulse2) / (float)Ts) * (60.0);
        //Calculate the speed in pulses per second and convert to RPM
        tmpPulse2 = enc2.getPulses();
        // KALKULASI SPEED AKHIR
        speedRPM2 = speedPulse2 / (float)PPR; // pulse/s / Pulse/rotation = rotation/s

        speedPulse3 = ((float)(enc3.getPulses() - tmpPulse3) / (float)Ts) * (60.0);
        //Calculate the speed in pulses per second and convert to RPM
        tmpPulse3 = enc3.getPulses();
        // KALKULASI SPEED AKHIR
        speedRPM3 = speedPulse3 / (float)PPR; // pulse/s / Pulse/rotation = rotation/s

        speedPulse4 = ((float)(enc4.getPulses() - tmpPulse4) / (float)Ts) * (60.0);
        //Calculate the speed in pulses per second and convert to RPM
        tmpPulse4 = enc4.getPulses();
        // KALKULASI SPEED AKHIR
        speedRPM4 = speedPulse4 / (float)PPR; // pulse/s / Pulse/rotation = rotation/s


        now = us_ticker_read();


    }

    // SET MOTOR SPEED
    motor.speed(0.5);

    // PRINT KE SERIAL
    printf("%f %f %f %f\n", speedRPM1, speedRPM2, speedRPM3, speedRPM4);
    // printf("hello world");
    }
}
*/ 



// main.cpp
#include "../../KRAI_Library/Motor/Motor.h"
#include "../../KRAI_Library/encoderKRAI/encoderKRAI.h"
#include "../../KRAI_Library/pidLo/pidLo.h" // Include the pidLo library
#include "../../KRAI_Library/pinout/F407VET6_2023.h"
// PIN Encoder
#define CHA F407VET6_ENCODER_1_4_A
#define CHB F407VET6_ENCODER_1_4_B
#define PPR 538

// PIN Motor
#define PWM F407VET6_PWM_MOTOR_2
#define FOR F407VET6_FOR_MOTOR_2
#define REV F407VET6_REV_MOTOR_2
// Define paramaeter PID
float Kp = 0.5;
float Ki = 0.5;
float Kd = 0;
float setpoint = 50; // SPEED TARGET (RPM)
float Ts = 100 * 0.001; // Sampling time (s)
// Define Object
encoderKRAI enc(CHA, CHB, PPR, Encoding::X4_ENCODING);
Motor motor(PWM, FOR, REV);
#define MAXOUT 1
#define VFF 0
#define RPF 1000
#define MAXIN 1000
pidLo pid(Kp, Ki, Kd, Ts, MAXOUT, VFF, RPF, MAXIN);

float speedRPM, speedPulse;
uint32_t now = us_ticker_read()/1000;
int32_t tmpPulse = enc.getPulses();

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
            /*
            // Parse the received string to extract KP, KI, KD values
            scanf("%f %f %f %f", &Kp, &Ki, &Kd, &setpoint);
            // Print the updated values
            printf("Updated PID values: Kp=%.2f, Ki=%.2f, Kd=%.2f, goal=%.2f\n", Kp, Ki, Kd, setpoint);
            // Set the updated PID values
            pid.setTunings(Kp, Ki, Kd);
            */

            scanf("%f", &motor_default_speed);
        }

        // Cek Speed berkala setiap Ts detik
        if (us_ticker_read()/1000 - now > Ts * 1000)
        {
            speedPulse = ((float)(enc.getPulses() - tmpPulse) / Ts) * (60.0); //
            //Calculate the speed in pulses per second and convert to RPM
            tmpPulse = enc.getPulses();
            now = us_ticker_read()/1000;

            // KALKULASI SPEED AKHIR
            speedRPM = speedPulse / PPR; // pulse/s / Pulse/rotation = rotation/s
            // Hitung ulang output yang dibutuhkan motor
            // Hitung berdasarkan delta state_sekarang dan state_target
            PID_error = setpoint - speedRPM;
            output = pid.createpwm(setpoint, speedRPM, 1.0); // Call the
            //createpwm() function from pidLo to get the output PWM value

            // SET MOTOR SPEED
            motor.speed(motor_default_speed);
            // Coba
            printf("%f ", output);
            printf("%f\n", speedRPM);
        }   
    }
}
