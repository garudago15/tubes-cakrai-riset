#include "mbed.h"
#include "../../../KRAI_Library/encoderKRAI/encoderKRAI.h"
#include "../../../KRAI_Library/Motor/Motor.h"
#include "../../../KRAI_Library/pidLo/pidLo.h"
#include "../../../KRAI_Library/JoystickPS3/JoystickPS3.h"
#include "../../../KRAI_Library/MovingAverage/MovingAverage.h"
#include "Constants.h"

// MOTOR
Motor motorFly(PWM_FLY, FWD_FLY, REV_FLY);
Motor motorAng(PWM_ANG, FWD_ANG, REV_ANG);
Motor motorRld(PWM_RLD, FWD_RLD, REV_RLD);
// ENCODER
encoderKRAI encFly(CHA_FLY, CHB_FLY, PPR_FLY, Encoding::X4_ENCODING);
encoderKRAI encAng(CHA_ANG, CHB_ANG, PPR_ANG, Encoding::X4_ENCODING);
encoderKRAI encRld(CHA_RLD, CHB_RLD, PPR_RLD, Encoding::X4_ENCODING);
// PID
pidLo pidFly(KP_FLY, KI_FLY, KD_FLY, (float)TS/1e3, MAXOUT, VFF, RPF, MAXIN);
pidLo pidAng(KP_ANG, KI_ANG, KD_ANG, (float)TS/1e3, MAXOUT, VFF, RPF, MAXIN);
// Moving Average
MovingAverage movAvgFly(20);
MovingAverage movAvgAng(20);
// Interrupt
InterruptIn LimitSwitch(LIMIT_SWITCH_PIN, PullDown);
JoystickPS3 ps3(PS3_TX, PS3_RX);

// SERIAL
static BufferedSerial serial_port(SERIAL_TX, SERIAL_RX, 115200);
FileHandle *mbed::mbed_override_console(int fd){
    return &serial_port;
}