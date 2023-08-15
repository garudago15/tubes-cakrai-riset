#include "mbed.h"
#include "ShooterMotor/ShooterMotor.h"
#include "Configurations/Constants.h"

// Setups
#include "Configurations/Setup.h"
ShooterMotor shooter(&motorFly, &motorRld, &motorAng, &encFly, &encRld, &encAng
    , &pidFly, &pidFly, &movAvgFly, &movAvgAng);

// Timing
uint32_t millis_ms(){
    return us_ticker_read()/1000;
}
uint32_t now = millis_ms();
uint32_t clock1 = millis_ms();

bool interruptState = false;
void riseMotor(){
    if(LimitSwitch.read()){
        interruptState = true;
        shooter.setIsReset(false);
        encAng.reset();
    }
}
void fallMotor(){
    if(!LimitSwitch.read()){
        interruptState = false;
    }
}

int main(){
    ps3.idle();
    ps3.setup();
    LimitSwitch.rise(&riseMotor);
    LimitSwitch.fall(&fallMotor);
    while(1){
        // Update Time
        now = millis_ms();

        // PS3
        ps3.olah_data();
        ps3.baca_data();

        shooter.setTargetRPM(2000);

        if(now - clock1 > TS){
            // Reloader
            if(ps3.getKotak()){
                shooter.reload();
            }
            shooter.controlRld();

            if(ps3.getL2()){
                shooter.setTargetANG(shooter.getTargetANG() + 1);
            }else if(ps3.getR2()){
                shooter.setTargetANG(shooter.getTargetANG() - 1);
            }
            // shooter.controlAng();

            // Fly
            shooter.controlFly();

            printf("%d | ", millis_ms());
            shooter.printData();

            clock1 = now;
        }
    }
}