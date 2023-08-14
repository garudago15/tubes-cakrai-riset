/**
 *  Header Encoder KRAI
 *  untuk membaca nilai rotary encoder
 **/
#ifndef ENCODERMOTORSPEED_H
#define ENCODERMOTORSPEED_H

//Bismillahirahamnirahim

//LIBRARY
#include "mbed.h"
#include "EncodingM.h"

/**************************
 * Konstanta dan Variabel *
 **************************/
 
//KONSTANTA
#define PREV_MASK 0x1 //Konstanta untuk mengetahui previous direction
#define CURR_MASK 0x2 //Konstanta untuk mengetahui current direction
#define INVALID   0x3 //XORing two states where both bits have changed.

/********************************
 * Quadrature Encoder Interface *
 ********************************/
 
class encoderMotorSpeed {
    private:
        void encode(void);
        /*******************************************
         * Menghitung pulse
         * Digunakan setiap rising/falling edge baik channel A atau B
         * Membaca putaran CW atau CCW => mengakibatkan pertambahan/pengurangan pulse
         *******************************************/
        
        //VARIABEL UNTUK PERHITUNGAN PULSE
        EncodingM encoding_;

        InterruptIn channelA_;
        InterruptIn channelB_interrupt_;
        DigitalIn channelB_digital_;

        int          pulsesPerRev_;
        int          prevState_;
        int          currState_;

        volatile int pulses_;
        volatile int lastPulses_;
        volatile int revolutions_;

    public:
        /******************************************* 
         * Membuat interface dari encoder    
         * @param inA DigitalIn, out A dari encoder
         * @param inB DigitalIn, out B dari encoder
         *******************************************/
        encoderMotorSpeed(PinName channelA, PinName channelB, int pulsesPerRev, EncodingM encoding = X2_ENCODING);
        
        /*******************************************
         * Reset encoder.
         * Reset pembacaaan menjadi 0
         *******************************************/
        void reset(void);
        
        /*******************************************
         * Membaca pulse yang didapat oleh encoder
         * @return Nilai pulse yang telah dilalui.
         *******************************************/
        int getPulses(void);
        
        uint32_t delta;
        uint32_t PPuS;
        uint32_t prevt=0;
        // int count_delta=0;
        /*******************************************
         * Membaca putaran yang didapat oleh encoder
         * @return Nilai revolusi/putaran yang telah dilalui.
         *******************************************/
        int getRevolutions(void);
};

#endif /* ENCODERKRAI_H */