/********************************************************
 * Main Program Elephant
 * Muhammad Fadhil Amri (KRU 14) & Hafizh Renanto Akhmad (KRU 14)
 * Last Modified : 23/04/2023
 ********************************************************/

// NOTE:
// Type 3: Top roller=> 2360 rpm, Bottom roller=> 4150 rpm
// Type 2 Depan: Top roller=> xxxx rpm, Bottom roller=> xxxx rpm : Target not reality
// Type 2 Belakang: Top roller=> xxxx rpm, Bottom roller=> xxxx rpm
// Type 1: Top roller=> 1900 rpm, Bottom roller=> 1900 rpm
// Type 1 Lawan: Top Roller => xxxx rpm, Bottom Roller => xxxx rpm

#include "mbed.h"
#include "Configurations/Variables.h"
#include "Configurations/Constants.h"
#include "../KRAI_Library/JoystickPS3/JoystickPS3.h"
#define DEBUG
#define NOPRINT
#include "utils.h"

void runMode(ThrowingMode throwingMode)
{
    pidTopThrower.reset();
    pidBottomThrower.reset();
    shooter.setMode(throwingMode);
    shooter.setRollerActive(true);
    hasChosenMode = true;
    // PRINTF("Mode chosen. %s\n", throwingMode.toString().c_str());
}

void releaseChoosingMode()
{
    hasChosenMode = false;
}

void stickTask()
{
    PRINTF("STICK START\n");

    /* SET UP JOYSTICK */
    stick.idle();
    stick.setup();
    if (switchConstant.read()){
        // High RED
        throwingModePoleType3.setSpeed(SPEED_POLE_TYPE_3_RED_TOP, SPEED_POLE_TYPE_3_RED_BOTTOM);
        throwingModePoleType3Miring.setSpeed(SPEED_POLE_TYPE_3_RED_MIRING_TOP, SPEED_POLE_TYPE_3_RED_MIRING_BOTTOM);
    } else {
        // Low BLUE 
        throwingModePoleType3.setSpeed(SPEED_POLE_TYPE_3_BLUE_TOP, SPEED_POLE_TYPE_3_BLUE_BOTTOM);
        throwingModePoleType3Miring.setSpeed(SPEED_POLE_TYPE_3_BLUE_MIRING_TOP, SPEED_POLE_TYPE_3_BLUE_MIRING_BOTTOM);
    }

    while (1)
    {


        /* Read Joystick every loop */
        stick.baca_data();

        // Memasuki autoShake
        if (!autoShakeReady && autoShakeAble && (( us_ticker_read()-lastShoot) > AUTO_SHAKE_DELAY))
        {
            autoShakeReady = true;
            autoShakeAble = false;

            lastReload = us_ticker_read();

            PRINTF("AutoShake Ready\n");
        }

        // Keluar dari autoshake
        if (autoShakeReady && (!autoShakeAble) && (( us_ticker_read()-lastReload) > AUTO_SHAKE_DURATION))
        {
            autoShakeReady = false;

            PRINTF("AutoShake Done\n");
        }

    


        if (stick.getStart())
        {
            NVIC_SystemReset();
        }

        vx_cmd = 0;
        vy_cmd = 0;
        w_cmd = 0;


        if (stick.getButtonRight()) {
            vx_cmd += TRANSLATION_BASE_SPEED;
        }

        if (stick.getButtonLeft()) {
            vx_cmd -= TRANSLATION_BASE_SPEED;
        }

        if (stick.getButtonUp()) {
            vy_cmd += TRANSLATION_BASE_SPEED;
        }

        if (stick.getButtonDown()) {
            vy_cmd -= TRANSLATION_BASE_SPEED;
        }

        if (stick.getR1()) {
            w_cmd -= ROTATION_BASE_SPEED;
        }

        if (stick.getL1()) {
            w_cmd += ROTATION_BASE_SPEED;
        }

        if (vx_cmd == 0.0 && vy_cmd == 0.0 && w_cmd == 0.0) {
            // Change controllers parameters if the base about to stop wholly (braking)
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

            if (stick.getR2()) {
                // Boost mode, also change controllers parameter
                vx_cmd *= TRANSLATION_BOOST_MULTIPLIER;
                vy_cmd *= TRANSLATION_BOOST_MULTIPLIER;
                w_cmd *= ROTATION_BOOST_MULTIPLIER;

                smcMotorBL.setKp(BASE_BL_SMC_KP_BOOST);
                smcMotorBR.setKp(BASE_BR_SMC_KP_BOOST);
                smcMotorFL.setKp(BASE_FL_SMC_KP_BOOST);
                smcMotorFR.setKp(BASE_FR_SMC_KP_BOOST);
            } else {
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

        // if (stick.getL2()) {
        //     vx_cmd *= TRANSLATION_HINDER_MULTIPLIER;
        //     vy_cmd *= TRANSLATION_HINDER_MULTIPLIER;
        //     w_cmd *= ROTATION_HINDER_MULTIPLIER;
        // }

        if (hasChosenMode)
        {
            if (!stick.getSegitiga() && !stick.getLingkaran() && !stick.getSilang() && !stick.getKotak() && !stick.getL2())
            {
                releaseChoosingMode();
            }
        }
        else
        {
            if (stick.getL2())
            {
                if (stick.getSegitiga())
                {
                    /* code */
                }
                else if (stick.getLingkaran())
                {
                    runMode(throwingModePoleType3Miring);
                    pidTopThrower.setTunings(TOP_ROLLER_TYPE3_MIRING_KC, TOP_ROLLER_TYPE3_MIRING_TAUI, TOP_ROLLER_TAUD);
                    pidBottomThrower.setTunings(BOTTOM_ROLLER_TYPE3_MIRING_KC, BOTTOM_ROLLER_TYPE3_MIRING_TAUI, BOTTOM_ROLLER_TAUD);
                }
                else if (stick.getSilang())
                {
                    runMode(throwingModePoleType2Close);
                    pidTopThrower.setTunings(TOP_ROLLER_TYPE2_CLOSE_KC, TOP_ROLLER_TYPE2_CLOSE_TAUI, TOP_ROLLER_TAUD);
                    pidBottomThrower.setTunings(BOTTOM_ROLLER_TYPE2_CLOSE_KC, BOTTOM_ROLLER_TYPE2_CLOSE_TAUI, BOTTOM_ROLLER_TAUD);
                }
                else if (stick.getKotak())
                {
                    runMode(throwingModePoleType1Close);
                    pidTopThrower.setTunings(TOP_ROLLER_TYPE1_CLOSE_KC, TOP_ROLLER_TYPE1_CLOSE_TAUI, TOP_ROLLER_TAUD);
                    pidBottomThrower.setTunings(BOTTOM_ROLLER_TYPE1_CLOSE_KC, BOTTOM_ROLLER_TYPE1_CLOSE_TAUI, BOTTOM_ROLLER_TAUD);
                } else if (stick.getSelect()){
                    autoShakeMode = true;
                    autoShakeAble = false;
                    autoShakeReady = false;
                }
            }
            else
            {
                if (stick.getSelect())
                {
                    if (stick.getSilang())
                    {
                        runMode(throwingModePoleType2Miring);
                        // pidTopThrower.setTunings(TOP_ROLLER_TYPE1_FAR_KC, TOP_ROLLER_TYPE1_FAR_TAUI, TOP_ROLLER_TAUD);
                        // pidBottomThrower.setTunings(BOTTOM_ROLLER_TYPE1_FAR_KC, BOTTOM_ROLLER_TYPE1_FAR_TAUI, BOTTOM_ROLLER_TAUD);
                        pidTopThrower.setTunings(TOP_ROLLER_TYPE2_MIRING_KC, TOP_ROLLER_TYPE2_MIRING_TAUI, TOP_ROLLER_TAUD);
                        pidBottomThrower.setTunings(BOTTOM_ROLLER_TYPE2_MIRING_KC, BOTTOM_ROLLER_TYPE2_MIRING_TAUI, BOTTOM_ROLLER_TAUD);
                    }
                    else if (stick.getKotak())
                    {
                        runMode(throwingModePoleType2Far);
                        pidTopThrower.setTunings(TOP_ROLLER_TYPE2_FAR_KC, TOP_ROLLER_TYPE2_FAR_TAUI, TOP_ROLLER_TAUD);
                        pidBottomThrower.setTunings(BOTTOM_ROLLER_TYPE2_FAR_KC, BOTTOM_ROLLER_TYPE2_FAR_TAUI, BOTTOM_ROLLER_TAUD);
                    } else if (stick.getR2()) {
                        autoShakeReady = false;
                        autoShakeAble = false;
                        autoShakeMode = false;
                        magazineReloader.setState(STATE_RELOADER_SHAKE_DOWN);
                    }
                }
                else if(stick.getLY() <= -64) // Analog Kiri naik
                {
                    if (stick.getKotak())
                    {
                        runMode(throwingModePoleType3Miring);
                        pidTopThrower.setTunings(TOP_ROLLER_TYPE3_MIRING_KC, TOP_ROLLER_TYPE3_MIRING_TAUI, TOP_ROLLER_TAUD);
                        pidBottomThrower.setTunings(BOTTOM_ROLLER_TYPE3_MIRING_KC, BOTTOM_ROLLER_TYPE3_MIRING_TAUI, BOTTOM_ROLLER_TAUD);
                    }

                } else
                {
                    if (stick.getKotak())
                    {
                        runMode(throwingModePoleType3);
                        pidTopThrower.setTunings(TOP_ROLLER_TYPE3_KC, TOP_ROLLER_TYPE3_TAUI, TOP_ROLLER_TAUD);
                        pidBottomThrower.setTunings(BOTTOM_ROLLER_TYPE3_KC, BOTTOM_ROLLER_TYPE3_TAUI, BOTTOM_ROLLER_TAUD);
                    }

                    if (stick.getLingkaran())
                    {
                        // PRINTF("lingkaran\n");
                        shooter.setRollerActive(false);
                    }
                    if (stick.getSilang()) // Ngeshoot
                    {
                        // PRINTF("kotak\n");
                        shooter.shoot();
                        ringPusher.Extend();
                        isReloadSafe = false;
                        isAfterShoot = true;
                        PRINTF("shoot\n");
                    }
                    else
                    {
                        ringPusher.Contract();
                        if(isAfterShoot){
                            lastShoot = us_ticker_read();
                            autoShakeAble = true;
                            PRINTF("after shoot\n");
                            isAfterShoot = false;
                        }
                        isReloadSafe = true;

                    }

                    if (stick.getSegitiga() && isReloadSafe) // Reloader => kalau lagi naik jadi turun dan sebaliknya
                    {
                        // PRINTF("Segitiga\n");
                        magazineReloader.toggleReloader();
                    }

                    if (((stick.getRY() <= -64) || (autoShakeReady && autoShakeMode)) && isReloadSafe) // Shake
                    {
                        // PRINTF("RY\n");
                        // magazine.position(-15);
                        // wait_us(200000);
                        // magazine.position(SERVO_DEFAULT_ANGLE);
                        // if (stick.getRY() < -100){
                        magazineReloader.setShakeDegree(SHAKE_DEGREE); // Mungkin nanti dibuat jadi adjustable
                        // } else {
                        //     magazineReloader.setShakeDegree(SHAKE_DEGREE-5); // Mungkin nanti dibuat jadi adjustable
                        // }
                        magazineReloader.shakeUp();

                    }

                    if ((stick.getRY() >= 0 && !autoShakeReady) && isReloadSafe )
                    {
                        magazineReloader.shakeDown();
                    }
                }
            }
        }

        ThisThread::sleep_for(1);
    }
}

void baseTask()
{
    // PRINTF("MASTER START\n");

    // pidAaronBerkMotorBL.setInputLimits(0, MAX_BL_INPUT_LIMIT);
    // pidAaronBerkMotorBR.setInputLimits(0, MAX_BR_INPUT_LIMIT);
    // pidAaronBerkMotorFL.setInputLimits(0, MAX_FL_INPUT_LIMIT);
    // pidAaronBerkMotorFR.setInputLimits(0, MAX_FR_INPUT_LIMIT);

    while (1)
    {
        if (us_ticker_read() - samplingStick > SAMP_STICK_US)
        {
            // stick.stickState(&vx_cmd, &vy_cmd, &w_cmd);

            controlElephant.set_vx_cmd(vx_cmd);
            controlElephant.set_vy_cmd(vy_cmd);
            controlElephant.set_w_cmd(w_cmd);

            // PRINTF("PS3: vx_cmd: %f, vy_cmd: %f, w_cmd: %f\n", vx_cmd, vy_cmd, w_cmd);

            samplingStick = us_ticker_read();
        }

        /********************** ENCODER **********************/

        if (us_ticker_read() - samplingEncoder > SAMP_BASE_MOTOR_ENCODER_US)
        {
            controlElephant.encoderMotorSamp();

            samplingEncoder = us_ticker_read();
        }

        /********************** PID & ODOM **********************/

        if (us_ticker_read() - samplingPID > SAMP_PID_BASE_MOTOR_US)
        {
            controlElephant.pidMotorSamp();

            samplingPID = us_ticker_read();
        }

        if (us_ticker_read() - samplingOdom > SAMP_BASE_ODOMETRY_US)
        {
            controlElephant.baseSpeed();

            samplingOdom = us_ticker_read();
        }

        if (us_ticker_read() - samplingUpdPos > SAMP_UPD_POS_US)
        {
            samplingUpdPos = us_ticker_read();

            controlElephant.updatePosition();
        }

        /********************** MOTOR **********************/

        if (us_ticker_read() - samplingMotor > SAMP_BASE_MOTOR_US)
        {
            samplingMotor = us_ticker_read();

            controlElephant.motorSamp();
        }

        /********************** IK **********************/

        if (us_ticker_read() - samplingIK > SAMP_IK_US)
        {
            controlElephant.base();

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

void reloaderTask()
{
    // PRINTF("RELOADER START\n");

    while (1)
    {
        magazineReloader.run();

        ThisThread::sleep_for(1);
    }
}

void shooterTask()
{
    // PRINTF("SHOOTER START\n");

    while (1)
    {
        if (magazineReloader.getState() != STATE_RELOADER_STANDBY_POSITION)
        {
            shooter.setmagazineReady(false);
        }
        else
        {
            shooter.setmagazineReady(true);
        }

        // Run Roller
        if (shooter.isRollerActive())
        {
            shooter.activateRoller();
        }
        else
        {
            shooter.setSpeedTargetRollerBottom(0);
            shooter.setSpeedTargetRollerTop(0);
            shooter.deactivateRoller();
        }

        shooter.run();

        ThisThread::sleep_for(10);
    }
}

int main()
{
    // PRINTF("START\n");

    // Setup serial_port
    // serial_port.set_format(
    //     /* bits */ 8,
    //     /* parity */ BufferedSerial::None,
    //     /* stop bit */ 1);

    // /* SET UP TIMER */
    us_ticker_init();

    serial_port.set_format(8, BufferedSerial::None, 1);
    threadBase.start(&baseTask);
    threadStick.start(&stickTask);
    threadReloader.start(&reloaderTask);
    threadShooter.start(&shooterTask);
    ringPusher.Contract();

    // Setup Input Limits roller
    pidTopThrower.setInputLimits(0, MAX_SPEED_ROLLER_TOP);
    pidBottomThrower.setInputLimits(0, MAX_SPEED_ROLLER_BOTTOM);

    while (1)
    {
        // MAIN LOGIC
        // topRoller.speed(0.8);
        // bottomRoller.speed(0.8);

        // Shooter
        // Nyala-matiin Roller

        // Message Passing dari reloader ke shooter

        // shooter.setRollerActive(true);

        // printf("topRollerSpeed: %f \t topRollerTarget: %f \t bottomRollerSpeed: %f \t bottomRollerTarget: %f\n", shooter.getRollerTopSpeed(), shooter.getRollerTopSpeedTarget(), shooter.getRollerBottomSpeed(), shooter.getRollerBottomSpeedTarget());
        // printf("%f,%f,%f,%f\n", shooter.getRollerTopSpeed(), shooter.getRollerTopSpeedTarget(), shooter.getRollerBottomSpeed(), shooter.getRollerBottomSpeedTarget());
        // PRINTF("PULSE rollerTop: %d rollerBottom: %d\n", encoderTopThrower.getPulses(), encoderBottomThrower.getPulses());
        // PRINTF("PULSE: %d\n", encoderBaseFL.getPulses());

        // ThisThread::sleep_for(10 * 1000); // 10 detik
        ThisThread::sleep_for(1ms);
    }
}
