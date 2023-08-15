#include "mbed.h"
#include "../../KRAI_library/Motor/Motor.h"
#include "../../KRAI_library/encoderKRAI/encoderKRAI.h"
#include "../../KRAI_library/Pinout/F407VET6_2023.h"
#include "../../KRAI_library/JoystickPS3/JoystickPS3.h"

static BufferedSerial serial_port(PA_9, PA_10, 115200);
FileHandle *mbed::mbed_override_console(int fd)
{
    return &serial_port;
}


// ---------------------------------------------------------------------

#define PWM F407VET6_PWM_MOTOR_7
#define FWD F407VET6_FOR_MOTOR_7
#define REV F407VET6_REV_MOTOR_7

#define CHA F407VET6_ENCODER_2_3_A
#define CHB F407VET6_ENCODER_2_3_B
#define PPR 105

#define LIMIT_SWITCH_PIN F407VET6_ENCODER_2_1_B

// --------------------------------------------------------------------

InterruptIn LimitSwitch(LIMIT_SWITCH_PIN, PullDown);
encoderKRAI enc(CHA ,CHB ,PPR ,Encoding::X4_ENCODING);

// --------------------------------------------------------------------

Motor motor(PWM, FWD, REV); 
int currentsudut = 0;

int millis_ms() {
    return (us_ticker_read() / 1000);
}

// -------------------------------------------------------------------

#define TS 50  // 50ms
int now; // time

float speedRPM;
float currentSudut, speedSudut;
float speedPulse, tmpPulse;
int tmp_sudut;

// ------------------------------------------------------------------

bool interruptState = false;
void riseMotor(){
    if(LimitSwitch.read()){
        interruptState = true;
        enc.reset();
    }
}
void fallMotor(){
    if(!LimitSwitch.read()){
        interruptState = false;
    }
}

// -------------- DEFINE STIK --------------
#define tx PA_2
#define rx PA_3
JoystickPS3 ps3(tx, rx);

// -----------------------------------------

// main() runs in its own thread in the OS
int main() {
    ps3.setup();                                    // Setup PS3
    LimitSwitch.rise(&riseMotor);
    LimitSwitch.fall(&fallMotor);
    now = millis_ms();
    tmpPulse = enc.getPulses();
    currentSudut = (enc.getPulses() * 360 / PPR);

    while (true) {
        
        ps3.olah_data();
        ps3.baca_data();

       
        // Looping

        while (millis_ms() -  now > TS)
        {
            speedPulse = (float(enc.getPulses() - tmpPulse)/TS) * 1000; // pulse / s
            tmpPulse = enc.getPulses(); //pulse
            now = millis_ms(); // ms
        }


        if (ps3.getL2())
        {
            motor.speed(0.3);
            printf("0.2");
        } else if (ps3.getR2()) {
            printf("-0.3");
            motor.speed(-0.4);
        } else {
            if (ps3.getSilang())
            {
                motor.forcebrake();
                printf("BREAK");
            } else {
                motor.brake(1);
            }
            
        }

        

        // motor.speed(0.2);
        
            

        speedRPM = speedPulse / PPR * 60; // pulse/s / Pulse/rotation = rotation/s
        speedSudut = speedPulse * 360 / PPR;
        printf("Pulse : %d Pulse Speed : %.2f Speed RPM : %.2f Spd Sdt : %.2f INT: %d\n", enc.getPulses(),speedPulse, speedRPM, speedSudut, interruptState);
        
    }
}