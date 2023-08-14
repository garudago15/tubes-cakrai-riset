#include "mbed.h"
#include "../../KRAI_Library/Motor/Motor.h"
#include "../../KRAI_Library/encoderKRAI/encoderKRAI.h"
#include "../../KRAI_Library/pidLo/pidLo.h" // Include the pidLo library
#include "../../KRAI_Library/Pinout/F407VET6_2023.h"

// PIN Encoder
#define CHA F407VET6_ENCODER_2_2_A
#define CHB F407VET6_ENCODER_2_2_B
#define PPR 538

// PIN Motor
#define PWM F407VET6_PWM_MOTOR_8
#define FOR F407VET6_FOR_MOTOR_8
#define REV F407VET6_REV_MOTOR_8

// Serial
#define RX_Serial PA_9
#define TX_Serial PA_10
static BufferedSerial serial_port(RX_Serial, TX_Serial, 115200);
FileHandle *mbed::mbed_override_console(int fd){
    return &serial_port;
}

// Define Object
encoderKRAI enc(CHA, CHB, PPR, Encoding::X4_ENCODING);
Motor motor(PWM, FOR, REV);

// Time
int timer1 = us_ticker_read();

// Interrupt
#define TEST_A_LIMIT_SWITCH F407VET6_ENCODER_2_1_B

InterruptIn limitSwitch_A(TEST_A_LIMIT_SWITCH, PullDown);
// DigitalIn limitSwitch_A(TEST_A_LIMIT_SWITCH, PullDown);
int state_A=0;
float motor_default_speed=0.3;

void callbackRise(){
    state_A=1;
    motor.forcebrake();
    enc.reset();
}
void callbackFall(){
    state_A=0;
}

int main(){
    limitSwitch_A.rise(&callbackRise);
    limitSwitch_A.fall(&callbackFall);

    printf("Start\n");
    

    while(1){
        // state_A = limitSwitch_A.read();

        if (us_ticker_read() - timer1 > 50000){
            if(state_A==0){
                motor.speed(motor_default_speed);
            }
            printf("%d %d\n", enc.getPulses(), state_A);
            // Dengan konfigurasi switch normally close dan PullDown resistor
            // Tanpa di-trigger, value dari limit switch adalah 1
            // Buat nge-test interrupt secara nyata, baiknya langsung diimplementasiin ke motor aslinya. Tentu saja code interruptnya belum nyampe sana :)
        }
    }
}