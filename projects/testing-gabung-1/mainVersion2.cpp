#include "Configs/Variables.h"

// ------------------------------------------------------------------

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

// --------------------------------------------------------------------



void shooterTask(){
    while(1){
        // --------------------------- ATUR SUDUT -----------------------
        if (ps3.getL2())
        {
            motorAngle.speed(0.3);
            printf("0.2 \n");
        } else if (ps3.getSelect()) { //sementara saja
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

        if (millis_ms() - now > Ts * 1000)
        {
            

            speedPulse = ((float)(encoderShooter.getPulses() - tmpPulse) / Ts) * (60.0); //
            //Calculate the speed in pulses per second and convert to RPM
            tmpPulse = encoderShooter.getPulses();

            speedRPM = speedPulse / PPR; // pulse/s / Pulse/rotation = rotation/s
            avgSpeedRPM = movAvg3.movingAverage(speedRPM);
            // Hitung ulang output yang dibutuhkan motor
            // Hitung berdasarkan delta state_sekarang dan state_target
            // PID_error = setpoint - speedRPM;
            avgPulses=movAvg2.movingAverage((float)encoderReloader.getPulses());
            currentSudut = (avgPulses * 360.0f / PPR);
            // output = pid.createpwm(setpoint, currentSudut, 1.0); // Call the
            // motorReloader.speed(output);
            //createpwm() function from pidLo to get the output PWM value

            if (currentSudut < setpoint){
                motorReloader.speed(reloader_speed);
            } else{
                motorReloader.forcebrake();
            }

            // printf("angle: %d\n", encoderAngle.getPulses());
            // printf("angle reloader: %.2f goal: %.2f rpmFly: %.2f Setpoint: %.5f ENC Sudut: %d I_State: %d\n", currentSudut, motor_default_speed, avgSpeedRPM, setpoint, encoderAngle.getPulses(), interruptState);
            // SET MOTOR SPEED
            // motor.speed(output);
            

            if(ps3.getLingkaran()){
                motorShooter.speed(motor_default_speed);
                printf("ON \n");
            } else if (ps3.getSegitiga()){
                motorShooter.speed(0);
                printf("OFF \n");
            }


            if(ps3.getKotak()){
                setpoint=180.0f;
                // printf("%d\n", 1);
            } else{
                // motorReloader.forcebrake();
                setpoint=0.0f;
                encoderReloader.reset();
                // printf("%d\n", 0);
            }
            
            // Coba
            // printf("%f ", output);
            
            // printf("%f, %f, %ld\n", speedRPM, output ,millis_ms()-SERIAL_PURPOSE_NOW);
            now = millis_ms();
        }   

        ThisThread::sleep_for(1);
    }
}


int main()
{
    // /* SET UP TIMER */
    us_ticker_init();

    // threadBase.start(&baseTask);
    // threadStick.start(&stickTask);
    // threadShooter.start(&shooterTask);

    LimitSwitch.rise(&riseMotor);
    LimitSwitch.fall(&fallMotor);

    printf("STICK START\n");

    /* SET UP JOYSTICK */
    ps3.idle();   
    ps3.setup();
    omni.encoderMotorSamp(); //baca data awal encoder omniWheel

    while (1)
    {
        // ThisThread::sleep_for(1ms);

        //----------------------------------------------------------
        // Stick Task
        //----------------------------------------------------------
        // Pengolahan dan Update data PS3
        ps3.olah_data();
        ps3.baca_data();

        //default
        vx_cmd = 0;
        vy_cmd = 0;
        w_cmd = 0;

        //braking system
        if(ps3.getStart()){ //tombol utk nembak
            omni.forceBrakeSync(); //hard brake sebelum nembak agar robot tidak gerak saat nembak
            omni.reset();
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

        //---------------------------------------
        //Base Task
        //---------------------------------------

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
            // printf("vFL: %.2f vFR: %.2f vBL: %.2f vBR: %.2f\n", omni.get_v_FL_curr(), omni.get_v_FR_curr(), omni.get_v_BL_curr(), omni.get_v_BR_curr());
            printf("FL: %d FR: %d BL:%d BR: %d shooter: %d reloader: %d angle: %d\n", encoderBaseFL.getPulses(), encoderBaseFR.getPulses(), encoderBaseBL.getPulses(), encoderBaseBR.getPulses(), encoderShooter.getPulses(), encoderReloader.getPulses(), encoderAngle.getPulses());

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

        if (!ps3.getKotak() && (us_ticker_read() - samplingMotor > SAMP_BASE_MOTOR_US)) //update motor
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

        //--------------------------------------------------------
        //Shooter Task
        //--------------------------------------------------------

        // --------------------------- ATUR SUDUT -----------------------
        if (ps3.getL2())
        {
            motorAngle.speed(0.3);
            printf("0.2 \n");
        } else if (ps3.getSelect()) { //sementara saja
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

        if (millis_ms() - now > Ts * 1000)
        {
            

            speedPulse = ((float)(encoderShooter.getPulses() - tmpPulse) / Ts) * (60.0); //
            //Calculate the speed in pulses per second and convert to RPM
            tmpPulse = encoderShooter.getPulses();

            speedRPM = speedPulse / PPR; // pulse/s / Pulse/rotation = rotation/s
            avgSpeedRPM = movAvg3.movingAverage(speedRPM);
            // Hitung ulang output yang dibutuhkan motor
            // Hitung berdasarkan delta state_sekarang dan state_target
            // PID_error = setpoint - speedRPM;
            avgPulses=movAvg2.movingAverage((float)encoderReloader.getPulses());
            currentSudut = (avgPulses * 360.0f / PPR);
            // output = pid.createpwm(setpoint, currentSudut, 1.0); // Call the
            // motorReloader.speed(output);
            //createpwm() function from pidLo to get the output PWM value

            if (currentSudut < setpoint){
                motorReloader.speed(reloader_speed);
            } else{
                motorReloader.forcebrake();
            }

            // printf("angle: %d\n", encoderAngle.getPulses());
            // printf("angle reloader: %.2f goal: %.2f rpmFly: %.2f Setpoint: %.5f ENC Sudut: %d I_State: %d\n", currentSudut, motor_default_speed, avgSpeedRPM, setpoint, encoderAngle.getPulses(), interruptState);
            // SET MOTOR SPEED
            // motor.speed(output);
            

            if(ps3.getLingkaran()){
                motorShooter.speed(motor_default_speed);
                printf("ON \n");
            } else if (ps3.getSegitiga()){
                motorShooter.speed(0);
                printf("OFF \n");
            }


            if(ps3.getKotak()){
                setpoint=180.0f;
                // printf("%d\n", 1);
            } else{
                // motorReloader.forcebrake();
                setpoint=0.0f;
                encoderReloader.reset();
                // printf("%d\n", 0);
            }
            
            // Coba
            // printf("%f ", output);
            
            // printf("%f, %f, %ld\n", speedRPM, output ,millis_ms()-SERIAL_PURPOSE_NOW);
            now = millis_ms();
        }   
    }
}
