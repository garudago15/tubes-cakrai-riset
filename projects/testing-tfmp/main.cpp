#include "mbed.h"
#include "../../KRAI_Library/Pinout/F446RE_MASTER_2022.h"
#include "../../KRAI_Library/TFMini/TFPlus.h"

// Pins and Configs
#define Serial_RX USBRX
#define Serial_TX USBTX
#define TFMP_SDA F446RE_MASTER_I2C_SDA
#define TFMP_SCL F446RE_MASTER_I2C_SCL

// Serial
static BufferedSerial serial_port(Serial_TX, Serial_RX, 115200);
FileHandle *mbed::mbed_override_console(int fd){
    return &serial_port;
}

// TFMini Plus Setup
const uint8_t TFMP_ADDR = 0x12;
TFplus TFMP(TFMP_SDA, TFMP_SCL, TFMP_ADDR);

int timer1 = us_ticker_read();
int timer2 = us_ticker_read();
int main(){
    // TFMP.findAddress();
    while(1){
        if(us_ticker_read()-timer1 > 1e6){
            TFMP.olahData();
            timer1 = us_ticker_read();
        }
        if(us_ticker_read()-timer2 > 1e6){
            printf("dist = %d, str = %d\n", TFMP.dist_HL, TFMP.strength_HL);
            timer2 = us_ticker_read();
        }
        // printf("test\n");
    }
}