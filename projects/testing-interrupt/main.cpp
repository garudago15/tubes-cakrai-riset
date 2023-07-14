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
#define TEST_LIMIT_SWITCH F407VET6_ENCODER_1_1_A
InterruptIn limitSwitch(TEST_LIMIT_SWITCH, PullDown);
void callbackPrint(){
    printf("\nLIMIT SWITCH CLICKED!!!\n");
}

int main(){
    limitSwitch.rise(&callbackPrint);
    printf("Start\n");
    while(1){
        if (us_ticker_read() - timer1 > 1e6){
            printf("Hello World\n");
        }
    }
}
