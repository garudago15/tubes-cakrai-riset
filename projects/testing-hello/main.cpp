#include "mbed.h"

// Serial
#define RX_Serial PA_9
#define TX_Serial PA_10
static BufferedSerial serial_port(RX_Serial, TX_Serial, 115200);
FileHandle *mbed::mbed_override_console(int fd){
    return &serial_port;
}

int timer1 = us_ticker_read();
int main(){
    printf("Start\n");
    while(1):
        if (us_ticker_read() - timer1 > 1e6){
            printf("Hello World\n");
        }
}
