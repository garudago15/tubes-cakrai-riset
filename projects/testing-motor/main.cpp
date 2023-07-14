#include "mbed.h"
#include "../../KRAI_Library/Motor/Motor.h"
#include "../../KRAI_Library/encoderKRAI/encoderKRAI.h"
#include "../../KRAI_Library/pinout/F407VET6_2023.h"

// PIN Encoder
#define CHA F407VET6_ENCODER_1_1_A
#define CHB F407VET6_ENCODER_1_1_B
#define PPR 538

// PIN Motor
#define PWM F407VET6_PWM_MOTOR_2
#define FOR F407VET6_FOR_MOTOR_2
#define REV F407VET6_REV_MOTOR_2

//variables
float speedPulse,tmpPulse,speedRPM;
uint32_t Ts=4173;

// Define Object
encoderKRAI enc(CHA, CHB, PPR, Encoding::X4_ENCODING);
Motor motor(PWM, FOR, REV);

/* INITIALIZE SERIAL USING PRINTF */
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
        speedPulse = ((float)(enc.getPulses() - tmpPulse) / Ts) * (60.0);
        //Calculate the speed in pulses per second and convert to RPM
        tmpPulse = (float)enc.getPulses();
        now = us_ticker_read();

        // KALKULASI SPEED AKHIR
        speedRPM = speedPulse / PPR; // pulse/s / Pulse/rotation = rotation/s
    }

    // SET MOTOR SPEED
    motor.speed(0.5);

    // PRINT KE SERIAL
    printf("%f\n", speedRPM);
    }
}