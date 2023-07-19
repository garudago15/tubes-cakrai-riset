// main.cpp
#include "mbed.h"
#include "../KRAI_Library/Motor/Motor.h"
#include "../KRAI_Library/encoderKRAI/encoderKRAI.h"
#include "../KRAI_Library/pidLo/pidLo.h" // Include the pidLo library
#include "../KRAI_Library/Pinout/F407VET6_2023.h"

// PIN Encoder
#define CHA F407VET6_ENCODER_2_3_A
#define CHB F407VET6_ENCODER_2_3_B
#define PPR 538

// PIN Motor
#define PWM F407VET6_PWM_MOTOR_1
#define FOR F407VET6_FOR_MOTOR_1
#define REV F407VET6_REV_MOTOR_1

// Define paramaeter PID
float Kp = 0.03;
float Ki = 0.8;
float Kd = 0.001;
float setpoint = 130; // SPEED TARGET (RPM) || MAX (2 Cell) => 150 rpm
float Ts = 50 * 0.001; // Sampling time (s)

// Define Object
encoderKRAI enc(CHA, CHB, PPR, Encoding::X4_ENCODING);
Motor motor(PWM, FOR, REV);
#define MAXOUT 1
#define VFF 0
#define RPF 1000
#define MAXIN 1000
pidLo pid(Kp, Ki, Kd, Ts, MAXOUT, VFF, RPF, MAXIN);

float speedRPM, speedPulse;
uint32_t millis_ms(){
    return us_ticker_read()/1000;
}
uint32_t now = millis_ms();
uint32_t SERIAL_PURPOSE_NOW = millis_ms();
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
            // Parse the received string to extract KP, KI, KD values
            scanf("%f %f %f %f", &Kp, &Ki, &Kd, &setpoint);
            // Print the updated values
            printf("Updated PID values: Kp=%.2f, Ki=%.2f, Kd=%.2f, goal=%.2f\n", Kp, Ki, Kd, setpoint);
            // Set the updated PID values
            pid.setTunings(Kp, Ki, Kd);

            // scanf("%f", &motor_default_speed);
        }

        // Cek Speed berkala setiap Ts detik
        if (millis_ms() - now > Ts * 1000)
        {
            speedPulse = ((float)(enc.getPulses() - tmpPulse) / Ts) * (60.0); //
            //Calculate the speed in pulses per second and convert to RPM
            tmpPulse = enc.getPulses();
            now = millis_ms();

            // KALKULASI SPEED AKHIR
            speedRPM = speedPulse / PPR; // pulse/s / Pulse/rotation = rotation/s
            // Hitung ulang output yang dibutuhkan motor
            // Hitung berdasarkan delta state_sekarang dan state_target
            PID_error = setpoint - speedRPM;
            output = pid.createpwm(setpoint, speedRPM, 1.0); // Call the
            //createpwm() function from pidLo to get the output PWM value

            // SET MOTOR SPEED
            motor.speed(output);

            // Coba
            // printf("%f ", output);
            printf("%f\n", speedRPM);
            // printf("%f, %f, %ld\n", speedRPM, output ,millis_ms()-SERIAL_PURPOSE_NOW);
        }   
    }
}