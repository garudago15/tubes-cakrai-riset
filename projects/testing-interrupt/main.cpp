#include "mbed.h"
#include "../../KRAI_Library/Pinout/F407VET6_2023.h"

// Serial
#define RX_Serial PA_9
#define TX_Serial PA_10
static BufferedSerial serial_port(RX_Serial, TX_Serial, 115200);
FileHandle *mbed::mbed_override_console(int fd){
    return &serial_port;
}

// Time
int timer1 = us_ticker_read();

// Interrupt
#define TEST_A_LIMIT_SWITCH F407VET6_ENCODER_1_2_A

// InterruptIn limitSwitch_A(TEST_A_LIMIT_SWITCH, PullDown);
DigitalIn limitSwitch_A(TEST_A_LIMIT_SWITCH, PullDown);

// void callbackPrint(){
//     Gak bisa printf dalam callback interrupt
// }

int main(){
    // limitSwitch.fall(&callbackPrint);

    printf("Start\n");
    int state_A;

    while(1){
        state_A = limitSwitch_A.read();

        if (us_ticker_read() - timer1 > 1e6){

            printf("%d\n", state_A);
            // Dengan konfigurasi switch normally close dan PullDown resistor
            // Tanpa di-trigger, value dari limit switch adalah 1
            // Buat nge-test interrupt secara nyata, baiknya langsung diimplementasiin ke motor aslinya. Tentu saja code interruptnya belum nyampe sana :)
        }
    }
}
