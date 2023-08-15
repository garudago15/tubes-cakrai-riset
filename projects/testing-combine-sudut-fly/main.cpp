// main.cpp
#include "mbed.h"
#include "../../libs/Configs/Constants.h"
#include "../../libs/Configs/ConfigurationPin.h"
#include "../../libs/MovingAverage/MovingAverage.h"
#include "../../KRAI_Library/Motor/Motor.h"
#include "../../KRAI_Library/encoderKRAI/encoderKRAI.h"
#include "../../KRAI_Library/pidLo/pidLo.h" // Include the pidLo library
#include "../../KRAI_Library/Pinout/F407VET6_2023.h"
#include "../../KRAI_Library/JoystickPS3/JoystickPS3.h"

// PIN Encoder
#define CHA1 F407VET6_ENCODER_2_4_A //shooter
#define CHB1 F407VET6_ENCODER_2_4_B

#define CHA2 F407VET6_ENCODER_2_2_A //reloader
#define CHB2 F407VET6_ENCODER_2_2_B

#define CHA3 F407VET6_ENCODER_2_3_A //sudut
#define CHB3 F407VET6_ENCODER_2_3_B

#define PPR 104.4

// PIN Motor
#define PWM1 F407VET6_PWM_MOTOR_5 //shooter
#define FOR1 F407VET6_FOR_MOTOR_5
#define REV1 F407VET6_REV_MOTOR_5

#define PWM2 F407VET6_PWM_MOTOR_8 //reloader
#define FOR2 F407VET6_FOR_MOTOR_8
#define REV2 F407VET6_REV_MOTOR_8

#define PWM3 F407VET6_PWM_MOTOR_7 //sudut
#define FOR3 F407VET6_FOR_MOTOR_7
#define REV3 F407VET6_REV_MOTOR_7

#define LIMIT_SWITCH_PIN F407VET6_ENCODER_2_1_B
InterruptIn LimitSwitch(LIMIT_SWITCH_PIN, PullDown);

// Define paramaeter PID
float Kp = 0.0f;
float Ki = 0.0f;
float Kd = 0.0f;
float setpoint = 0.0f; // SPEED TARGET (RPM) || MAX (2 Cell) => 150 rpm
float Ts = 5 * 0.001; // Sampling time (s)

// Define Object
encoderKRAI enc1(CHA1, CHB1, PPR, Encoding::X4_ENCODING);
Motor motor1(PWM1, FOR1, REV1);
encoderKRAI enc2(CHA2, CHB2, PPR, Encoding::X4_ENCODING);
Motor motor2(PWM2, FOR2, REV2);
encoderKRAI enc3(CHA3, CHB3, PPR, Encoding::X4_ENCODING);
Motor motor3(PWM3, FOR3, REV3);

#define MAXOUT 1
#define VFF 0
#define RPF 1000
#define MAXIN 1000

pidLo pid(Kp, Ki, Kd, Ts, MAXOUT, VFF, RPF, MAXIN);
MovingAverage movAvg2(20);
MovingAverage movAvg3(20);

/* INITIALIZE PS3 JOYSTICK */
JoystickPS3 ps3(UART_STICK_TX, UART_STICK_RX);

float speedRPM, speedPulse, avgSpeedRPM;
uint32_t millis_ms(){
    return us_ticker_read()/1000;
}
uint32_t now = millis_ms();
uint32_t SERIAL_PURPOSE_NOW = millis_ms();
int32_t tmpPulse = enc1.getPulses();

float PID_error, output;
float motor_default_speed=0.7, reloader_speed=0.6;
float currentSudut, avgPulses;

// INITIALIZE SERIAL USING PRINTF 
static BufferedSerial serial_port(PA_9, PA_10, 115200);
FileHandle *mbed::mbed_override_console(int fd){
    return &serial_port;
}

// --------------------------------------------------------------------



// ------------------------------------------------------------------

bool interruptState = false;
void riseMotor(){
    if(LimitSwitch.read()){
        interruptState = true;
        enc3.reset();
    }
}
void fallMotor(){
    if(!LimitSwitch.read()){
        interruptState = false;
    }
}

// --------------------------------------------------------------------

int main()
{
    ps3.idle();   
    ps3.setup();

    LimitSwitch.rise(&riseMotor);
    LimitSwitch.fall(&fallMotor);

    while (true)
    {
        ps3.olah_data();
        ps3.baca_data();
        // Check for incoming serial data
        if (serial_port.readable())
        {
            scanf("%f %f", &motor_default_speed, &reloader_speed);
            // scanf("%f %f %f", &Kp, &Ki, &Kd);
            // pid.setTunings(Kp, Ki, Kd);
        }

        // --------------------------- ATUR SUDUT -----------------------
        if (ps3.getL2())
        {
            motor3.speed(0.3);
            printf("0.2 \n");
        } else if (ps3.getR2()) {
            printf("-0.3 \n");
            motor3.speed(-0.4);
        } else {
            if (ps3.getSilang())
            {
                motor3.forcebrake();
                printf("BREAK \n");
            } else {
                motor3.brake(1);
            }
            
        }
        //-----------------------------------------------------------

        // Cek Speed berkala setiap Ts detik
        if (millis_ms() - now > Ts * 1000)
        {
            

            speedPulse = ((float)(enc1.getPulses() - tmpPulse) / Ts) * (60.0); //
            //Calculate the speed in pulses per second and convert to RPM
            tmpPulse = enc1.getPulses();

            speedRPM = speedPulse / PPR; // pulse/s / Pulse/rotation = rotation/s
            avgSpeedRPM = movAvg3.movingAverage(speedRPM);
            // Hitung ulang output yang dibutuhkan motor
            // Hitung berdasarkan delta state_sekarang dan state_target
            // PID_error = setpoint - speedRPM;
            avgPulses=movAvg2.movingAverage((float)enc2.getPulses());
            currentSudut = (avgPulses * 360.0f / PPR);
            output = pid.createpwm(setpoint, currentSudut, 1.0); // Call the
            // motor2.speed(output);
            //createpwm() function from pidLo to get the output PWM value

            if (currentSudut < setpoint){
                motor2.speed(reloader_speed);
            } else{
                motor2.forcebrake();
            }

            // printf("angle: %d\n", enc3.getPulses());
            printf("angle reloader: %.2f goal: %.2f pwm: %.2f rpmFly: %.2f Setpoint: %.5f ENC Sudut: %d I_State: %d\n", currentSudut, motor_default_speed, output, avgSpeedRPM, setpoint, enc3.getPulses(), interruptState);
            // SET MOTOR SPEED
            // motor.speed(output);
            

            if(ps3.getLingkaran()){
                motor1.speed(motor_default_speed);
                printf("ON \n");
            } else if (ps3.getSegitiga()){
                motor1.speed(0);
                printf("OFF \n");
            }


            if(ps3.getKotak()){
                setpoint=180.0f;
                // printf("%d\n", 1);
            } else{
                // motor2.forcebrake();
                setpoint=0.0f;
                enc2.reset();
                // printf("%d\n", 0);
            }
            
            // Coba
            // printf("%f ", output);
            
            // printf("%f, %f, %ld\n", speedRPM, output ,millis_ms()-SERIAL_PURPOSE_NOW);
            now = millis_ms();
        }   
    }
}