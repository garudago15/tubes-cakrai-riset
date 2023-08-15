#include "Configs/Variables.h"

bool interruptState = false;
void riseMotor(){
    if(LimitSwitch.read()){
        interruptState = true;
        encoderAngle.reset();
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
        ps3.olah_data();
        ps3.baca_data();

        if (serial_port.readable())
        {
            scanf("%f", &motor_default_speed);
            printf("%.2f", motor_default_speed);
            // scanf(" %c", &letter);
            // letter=getchar();
        }
        
        if(us_ticker_read() - samplingEncoder > SAMP_BASE_MOTOR_ENCODER_US){
            printf("FL: %d FR: %d BL:%d BR: %d shooter: %d reloader: %d angle: %d\n", encoderBaseFL.getPulses(), encoderBaseFR.getPulses(), encoderBaseBL.getPulses(), encoderBaseBR.getPulses(), encoderShooter.getPulses(), encoderReloader.getPulses(), encoderAngle.getPulses());
            
            baseMotorBL.speed(motor_default_speed);
            baseMotorBR.speed(motor_default_speed);
            baseMotorFL.speed(motor_default_speed);
            baseMotorFR.speed(motor_default_speed);

            motorShooter.speed(motor_default_speed);
            motorReloader.speed(motor_default_speed);

            if (ps3.getL2())
            {
                motorAngle.speed(0.3);
                printf("0.2 \n");
            } else if (ps3.getR2()) { 
                printf("-0.3 \n");
                motorAngle.speed(-0.4);
            } else {
                if (ps3.getSilang())
                {
                    motorAngle.forcebrake();
                    printf("BREAK \n");
                } else {
                    motorAngle.brake(1);
                }
            }

            samplingEncoder=us_ticker_read();
        }
    }
    
    return 0;
}