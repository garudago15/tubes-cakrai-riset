
#ifndef NO_PWM_MOTOR_H
#define NO_PWM_MOTOR_H

#include "mbed.h"

#define BRAKE_HIGH 1
#define BRAKE_LOW  0

class noPWMMotor {
public:

    /** Create a motor control interface    
     *
     * @param pwm A PwmOut pin, driving the H-bridge enable line to control the speed
     * @param fwd A DigitalOut, set high when the motor should go forward
     * @param rev A DigitalOut, set high when the motor should go backwards
     */
    noPWMMotor(PinName pwm, PinName fwd, PinName rev);
    
    /** Set the onOff of the motor
     * 
     * @param onOff The speed of the motor as an int value (-1, 0, or 1)
     */
    void onOff(int onOff);
    
    /** Brake the H-bridge to GND or VCC.
     * 
     * Defaults to breaking to VCC.
     *
     * Brake to GND => inA = inB = 0
     * Brake to VCC => inA = inB = 1
     */
    void brake(int highLow = BRAKE_HIGH);
    
    /**
    *Brake lebih baik
    **/
    void forcebrake();

protected:
    DigitalOut _pwm;
    DigitalOut _fwd;
    DigitalOut _rev;
    

};

#endif