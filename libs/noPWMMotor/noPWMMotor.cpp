
#include "noPWMMotor.h"

noPWMMotor::noPWMMotor(PinName pwm, PinName fwd, PinName rev):
        _pwm(pwm), _fwd(fwd), _rev(rev) {

    // Initial condition of output enables
    _pwm = 0;
    _fwd = 0;
    _rev = 0;
}

void noPWMMotor::onOff(int onOff) {
    _fwd = (onOff > 0);
    _rev = (onOff < 0);
    _pwm = abs(onOff);
}

void noPWMMotor::brake(int highLow){

    if(highLow == BRAKE_HIGH){
        _fwd = 1;
        _rev = 1;
    }
    else if(highLow == BRAKE_LOW){
        _fwd = 0;
        _rev = 0;
    }

}

void noPWMMotor::forcebrake(){
    _pwm = 1;
    _fwd = 1;
    _rev = 1;
}
