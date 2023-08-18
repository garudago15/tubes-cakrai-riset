#include "led4Pin-digital.h"

LED4PinDigital::LED4PinDigital(PinName red, PinName green, PinName blue):
        _red(red), _green(green), _blue(blue) {

    // Set initial condition of PWM
    _red = 0;
    _green = 0;
    _blue = 0;

}

void LED4PinDigital::setRGB(bool redBool, bool greenBool, bool blueBool){
    
    _red = redBool;
    _green = greenBool;
    _blue = blueBool;
}

void LED4PinDigital::setColor(string strColor){
    // RED(255,0,0), GREEN(0,255,0), BLUE(0,0,255)
    // PURPLE(200,0,255), CYAN(0,255,255), YELLOW(255,255,0), ORANGE(255, 125, 0)

    if(strColor == "RED"){
        setRGB(true, false, false);
    } else if (strColor == "GREEN"){
        setRGB(false, true, false);
    } else if (strColor == "BLUE"){
        setRGB(false, false, true);
    } else if (strColor == "PURPLE"){
        setRGB(true, false, true);
    } else if (strColor == "CYAN"){
        setRGB(false, true, true);
    } else if (strColor == "YELLOW"){
        setRGB(true, true, false);
    } 
    
}

void LED4PinDigital::turnOff(){
    setRGB(false, false, false);
}