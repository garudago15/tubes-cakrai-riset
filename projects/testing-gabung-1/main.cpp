#include "Configs/Variables.h"

void stickTask(){
    printf("STICK START\n");

    /* SET UP JOYSTICK */
    ps3.idle();   
    ps3.setup();
    omni.encoderMotorSamp(); //baca data awal encoder omniWheel
    while (true){
        // Pengolahan dan Update data PS3
        ps3.olah_data();
        ps3.baca_data();
        
        // utk reset
        if (ps3.getStart())
        {
            NVIC_SystemReset();
            printf("start");
        }

        //default
        vx_cmd = 0;
        vy_cmd = 0;
        w_cmd = 0;

        //braking system
        if(ps3.getKotak()){ //tombol utk nembak
            omni.forceBrakeSync(); //hard brake sebelum nembak agar robot tidak gerak saat nembak
        } else{
            //moving
            if (ps3.getButtonRight()) { //gerak ke kanan
                vx_cmd += TRANSLATION_BASE_SPEED;
            } 
            if (ps3.getButtonLeft()) { //gerak ke kiri
                vx_cmd -= TRANSLATION_BASE_SPEED;
            } 

            if (ps3.getButtonUp()) { //gerak ke atas
                vy_cmd += TRANSLATION_BASE_SPEED;
            } 
            if (ps3.getButtonDown()) { //gerak ke bawah
                vy_cmd -= TRANSLATION_BASE_SPEED;
            }

            if (ps3.getL1()) { //spin berlawanan arah jarum jam
                w_cmd += ROTATION_BASE_SPEED;
            } 
            if (ps3.getR1()) { //spin berlawanan arah jarm jam
                w_cmd -= ROTATION_BASE_SPEED;
            }

            if (vx_cmd == 0.0 && vy_cmd == 0.0 && w_cmd == 0.0) {
                // Change controllers parameters if the base about to stop wholly (soft braking)
                smcMotorBL.setKp(BASE_BL_SMC_KP_BRAKE);
                smcMotorBR.setKp(BASE_BR_SMC_KP_BRAKE);
                smcMotorFL.setKp(BASE_FL_SMC_KP_BRAKE);
                smcMotorFR.setKp(BASE_FR_SMC_KP_BRAKE);

                smcMotorBL.setKsigma(SMC_BL_KSIGMA_BRAKE);
                smcMotorBR.setKsigma(SMC_BR_KSIGMA_BRAKE);
                smcMotorFL.setKsigma(SMC_FL_KSIGMA_BRAKE);
                smcMotorFR.setKsigma(SMC_FR_KSIGMA_BRAKE);

                pidMotorBL.setKp(BASE_BL_KP_BRAKE);
                pidMotorBR.setKp(BASE_BR_KP_BRAKE);
                pidMotorFL.setKp(BASE_FL_KP_BRAKE);
                pidMotorFR.setKp(BASE_FR_KP_BRAKE);    
            } else {
                // Change back controller parameters if base is moving

                if (ps3.getR2()) {
                    // Boost mode, also change controllers parameter
                    vx_cmd *= TRANSLATION_BOOST_MULTIPLIER;
                    vy_cmd *= TRANSLATION_BOOST_MULTIPLIER;
                    w_cmd *= ROTATION_BOOST_MULTIPLIER;

                    smcMotorBL.setKp(BASE_BL_SMC_KP_BOOST);
                    smcMotorBR.setKp(BASE_BR_SMC_KP_BOOST);
                    smcMotorFL.setKp(BASE_FL_SMC_KP_BOOST);
                    smcMotorFR.setKp(BASE_FR_SMC_KP_BOOST);
                } else { //normal mode
                    smcMotorBL.setKp(BASE_BL_SMC_KP);
                    smcMotorBR.setKp(BASE_BR_SMC_KP);
                    smcMotorFL.setKp(BASE_FL_SMC_KP);
                    smcMotorFR.setKp(BASE_FR_SMC_KP);
                }

                smcMotorBL.setKsigma(SMC_BL_KSIGMA);
                smcMotorBR.setKsigma(SMC_BR_KSIGMA);
                smcMotorFL.setKsigma(SMC_FL_KSIGMA);
                smcMotorFR.setKsigma(SMC_FR_KSIGMA);

                pidMotorBL.setKp(BASE_BL_KP);
                pidMotorBR.setKp(BASE_BR_KP);
                pidMotorFL.setKp(BASE_FL_KP);
                pidMotorFR.setKp(BASE_FR_KP);
            }
        }
        ThisThread::sleep_for(1);
    }
}

void baseTask(){
    while(1){
        //stick sampling
        if (us_ticker_read() - samplingStick > SAMP_STICK_US)
        {
            // stick.stickState(&vx_cmd, &vy_cmd, &w_cmd);

            omni.set_vx_cmd(vx_cmd);
            omni.set_vy_cmd(vy_cmd);
            omni.set_w_cmd(w_cmd);

            // printf("PS3: vx_cmd: %f, vy_cmd: %f, w_cmd: %f\n", vx_cmd, vy_cmd, w_cmd);

            samplingStick = us_ticker_read();
        }

        /********************** ENCODER **********************/

        if (us_ticker_read() - samplingEncoder > SAMP_BASE_MOTOR_ENCODER_US)
        {
            omni.encoderMotorSamp();

            //printing curr speed
            // float junk1;
            
            // omni.getVars(&junk1,&junk1,&junk1,&v_FL_curr,&v_FR_curr,&v_BR_curr,&v_BL_curr,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1,&junk1);
            printf("vFL: %.2f vFR: %.2f vBL: %.2f vBR: %.2f\n", omni.get_v_FL_curr(), omni.get_v_FR_curr(), omni.get_v_BL_curr(), omni.get_v_BR_curr());
            // printf("FL: %d FR: %d BL:%d BR: %d\n", encoderBaseFL.getPulses(), encoderBaseFR.getPulses(), encoderBaseBL.getPulses(), encoderBaseBR.getPulses());

            samplingEncoder = us_ticker_read();
        }

        /********************** PID & ODOM **********************/

        if (us_ticker_read() - samplingPID > SAMP_PID_BASE_MOTOR_US) //update PID berdasarkan target dan current speed tiap motor
        {
            omni.pidMotorSamp();

            samplingPID = us_ticker_read();
        }

        if (us_ticker_read() - samplingOdom > SAMP_BASE_ODOMETRY_US) //update odometri
        {
            omni.baseSpeed();

            samplingOdom = us_ticker_read();
        }

        /********************** MOTOR **********************/

        if (us_ticker_read() - samplingMotor > SAMP_BASE_MOTOR_US) //update motor
        {
            samplingMotor = us_ticker_read();

            omni.motorSamp();
        }

        /********************** IK **********************/

        if (us_ticker_read() - samplingIK > SAMP_IK_US) //update target speed tiap motor berdasarkan vx, vy, dan w
        {
            omni.base();

            samplingIK = us_ticker_read();
        }
        /********************** PRINTS **********************/

        // // Jalanin semua roda di base
        // baseMotorBL.speed(0.4);
        // baseMotorBR.speed(0.4);
        // baseMotorFL.speed(0.4);
        // baseMotorFR.speed(0.4);

        // PRINTF("FR : %d, FL : %d, BR : %d, BL : %d\n", encoderBaseFR.getPulses(), encoderBaseFL.getPulses(), encoderBaseBR.getPulses(), encoderBaseBL.getPulses());
        ThisThread::sleep_for(1);
    }
}

void shooterTask(){
    while(1){
        ThisThread::sleep_for(10);
    }
}


int main()
{
    // /* SET UP TIMER */
    us_ticker_init();

    threadBase.start(&baseTask);
    threadStick.start(&stickTask);
    threadShooter.start(&shooterTask);

    while (1)
    {
        ThisThread::sleep_for(1ms);
    }
}
