/*
    Library untuk mengontrol LED RGB Streep 4 pin (commmon anode) KHUSUS UNTUK DIGITAL

 */

#ifndef MBED_LED4_RGB_DIGITAL
#define MBED_LED4_RGB_DIGITAL

#include "mbed.h"
#include <math.h>


class LED4PinDigital {
public:

    /** 
     *
     * @param red
     * @param green
     * @param blue
     * 
     */
    LED4PinDigital(PinName red, PinName green, PinName blue);
    
    /*
    FUNGSI setRGB(bool red255, bool green255, bool blue255);
    Melakukan perubah warna LED RGB berdasarkan kode RGB skala 0-255
    */
    void setRGB(bool redBool, bool greenBool, bool blueBool);

    /*
    FUNGSI setColor(string strColor);
    Melakukan perubahan warna dengan warna yang sudah di dekrasi, berikut list warna :
        - RED(255,0,0), GREEN(0,255,0), BLUE(0,0,255)
        - PURPLE(200,0,255), CYAN(0,255,255), YELLOW(255,255,0), ORANGE(255, 125, 0)
    */
    void setColor(string strColor);

    /*
    FUNGSI turnOff();
    Mematikan LED
    */
    void turnOff();

protected:
    DigitalOut _red;
    DigitalOut _green;
    DigitalOut _blue;

};

#endif