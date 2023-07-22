// main.cpp
#include "mbed.h"
#include "../../KRAI_Library/Motor/Motor.h"
#include "../../KRAI_Library/encoderKRAI/encoderKRAI.h"
#include "../../KRAI_Library/pidLo/pidLo.h" // Include the pidLo library
#include "../../KRAI_Library/Pinout/F446RE_SLAVE_2022.h"
#include "../../KRAI_Library/MovingAverage/MovingAverage.h"

// PIN Encoder
#define CHA F446RE_SLAVE_ENCODER_1_B_A
#define CHB F446RE_SLAVE_ENCODER_1_B_B
#define PPR 538

// PIN Motor
#define PWM F446RE_SLAVE_PWM_MOTOR_A
#define FOR F446RE_SLAVE_FOR_MOTOR_A
#define REV F446RE_SLAVE_REV_MOTOR_A

// Define paramaeter PID
float Kp = 0.03;
float Ki = 0.8;
float Kd = 0.001;
float setpoint = 130; // SPEED TARGET (RPM) || MAX (2 Cell) => 150 rpm
float Ts = 5 * 0.001; // Sampling time (s)

// Define Object
encoderKRAI enc(CHA, CHB, PPR, Encoding::X4_ENCODING);
Motor motor(PWM, FOR, REV);
#define MAXOUT 1
#define VFF 0
#define RPF 1000
#define MAXIN 1000
pidLo pid(Kp, Ki, Kd, Ts, MAXOUT, VFF, RPF, MAXIN);
MovingAverage movAvg(20); // Banyaknya data yang dirata-rata dalam satu waktu adalah 4

float speedRPM, speedPulse, speedPulseAvg;
uint32_t millis_ms(){
    return us_ticker_read()/1000;
}
uint32_t now = millis_ms();
uint32_t SERIAL_PURPOSE_NOW = millis_ms();
int32_t tmpPulse = enc.getPulses();

float PID_error, output;
float motor_default_speed=0.5;

// INITIALIZE SERIAL USING PRINTF 
static BufferedSerial serial_port(USBTX, USBRX, 115200);
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
            // speedPulse = ((float)(enc.getPulses() - tmpPulse) / Ts) * (60.0); //
            //Calculate the speed in pulses per second and convert to RPM

            speedPulseAvg = movAvg.movingAverage(((float)(enc.getPulses() - tmpPulse) / Ts) * (60.0));
            // printf("%f, %f\n", (float)(enc.getPulses() - tmpPulse), pulseAvg);


            tmpPulse = enc.getPulses();
            now = millis_ms();

            // KALKULASI SPEED AKHIR
            speedRPM = speedPulseAvg / PPR; // pulse/s / Pulse/rotation = rotation/s

            // Hitung ulang output yang dibutuhkan motor
            // Hitung berdasarkan delta state_sekarang dan state_target
            PID_error = setpoint - speedRPM;
            output = pid.createpwm(setpoint, speedRPM, 1.0); // Call the
            //createpwm() function from pidLo to get the output PWM value

            // SET MOTOR SPEED
            // motor.speed(output);
            motor.speed(1);

            // Coba
            printf("%f\n", speedRPM);
            // printf("%f, %d\n", speedRPM, millis_ms() - SERIAL_PURPOSE_NOW);
            // printf("%f, %f\n", speedRPM, setpoint);

            // printf("%f ", output);
            // movAvgRPM = movAvg.movingAverage(speedRPM);
            // printf("%f, %f, %ld\n", speedRPM, output ,millis_ms()-SERIAL_PURPOSE_NOW);
        }   
    }
}