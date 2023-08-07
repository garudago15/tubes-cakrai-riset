/********************************************************
 * Main Program Rabbit
 * Muhammad Fadhil Amri (KRU 14) & Hafizh Renanto Akhmad (KRU 14)
 * Last Modified : 23/04/2023
 ********************************************************/

#include "mbed.h"

#include "Configurations/Variables.h"
#include "Configurations/Constants.h"
#include "../KRAI_Library/JoystickPS3/JoystickPS3.h"

#include <string>
#include <stdexcept>

using namespace std;

// Tidak bisa kalau tidak di main.cpp
/** START: DECLARING SPI **/
// SPI spi_m(SPI_MOSI, SPI_MISO, SPI_SCK); // mosi, miso, sclk
// DigitalOut cs_m(SPI_SS);
/** END: DECLARING SPI **/

/* SERIAL PORT */
static BufferedSerial serial_port(PA_9, PA_10, 115200);
FileHandle *mbed::mbed_override_console(int fd)
{
    return &serial_port;
}

/* THREAD FOR ULTRASONIC (STILL NAIVE) */
Thread ultrasonicRThread;
Thread ultrasonicLThread;

// Timer ultrasonicRTimer;
// Timer ultrasonicLTimer;

// void ultrasonicR_read() {
//     while (true)
//     {
//         /* code */
//         if (ultrasonicRTimer.read() > 0.005) {
//             ultrasonicR.sensor();
//             ultrasonicRTimer.reset();
//         }
//     }

// }
// void ultrasonicL_read() {
//     while (true)
//     {
//         /* code */
//         if (ultrasonicLTimer.read() > 0.005) {
//             ultrasonicL.sensor();
//             ultrasonicLTimer.reset();
//         }
//     }

// }

int main()
{
    // printf("SPI START\n");
    // printf("Master Rabbit\n");
    // ultrasonicRTimer.start();
    // ultrasonicLTimer.start();
    // ultrasonicRThread.start(&ultrasonicR_read);
    // ultrasonicLThread.start(&ultrasonicL_read);

    // spi_m.frequency(INIT_SCK);
    // spi_m.format(8, 3);
    // cs_m = 1;


    led = 0;

    /* SETTING UP SERIAL PORT */
    serial_port.set_format(
        /* bits */ 8,
        /* parity */ BufferedSerial::None,
        /* stop bit */ 1);

    /* INITIALIZE TIMER */
    us_ticker_init();

    /* SETTING UP ODOMETRY */
    odom.resetOdom();

    /* SETTING UP JOYSTICK */
    stick.setup();
    stick.idle();

    /** MAIN LOOP **/
    while (1)
    {
        pidAaronBerkMotorBL.setInputLimits(0, 3.0);
        pidAaronBerkMotorBR.setInputLimits(0, 3.0);
        pidAaronBerkMotorFL.setInputLimits(0, 3.0);
        pidAaronBerkMotorFR.setInputLimits(0, 3.0);

        /********************** STICK **********************/
        if (stick.getStart())
        {
            NVIC_SystemReset();
        }

        // if (stick.getL2())
        // {
        //     resetMekanisme = 0;
        // } else {
        //     resetMekanisme = 1;
        // }

        /* READ JOYSTICK */
        stick.baca_data();
        if (stick.getButtonUp()){
            led = 1;
        } else if (stick.getButtonDown()){
            led = 0;
        }

        /************************ OTOMATIS *************************/
        // if (us_ticker_read() - samplingOtomatis > SAMP_OTOM_US)
        // {
        //     if (stick.getMode() == true)
        //     { // kalau L2
        //         // printf("Mode 1\n");
        //         if (controlRabbit.getOtomatis() == false)
        //         {
        //             controlRabbit.setOtomatis(true);
        //             // printf("mode otomatis\n");
        //         }
        //         else
        //         {
        //             controlRabbit.setOtomatis(false);
        //         }
        //         samplingOtomatis = us_ticker_read();
        //     }
        // }

        /********************** PARALLEL PARK **********************/
        // if (us_ticker_read() - samplingParallelPark > SAMP_PARALLEL_PARK_US)
        // {
        //     if (stick.parallelParkMode())
        //     { // kalau R2
        //         printf("parallel park mode MASUK IF\n");
        //         // controlRabbit.setRobotMode(!controlRabbit.getOffsetFromPoleMode());
        //         controlRabbit.setRobotToPoleMiddleMode(!controlRabbit.getRobotToPoleMiddleMode());

        //         // samplingParallelPark = us_ticker_read();
        //     }

        //     if (stick.offsetFromPoleMode()) {
        //         controlRabbit.setOffsetFromPoleMode(!controlRabbit.getOffsetFromPoleMode());
        //         // printf("offset from pole mode");
        //     }
        //     samplingParallelPark = us_ticker_read();
        // }

        if (us_ticker_read() - samplingStick > SAMP_STICK_US)
        {

            float vx_cmd, vy_cmd, w_cmd;

            stick.stickState(&vx_cmd, &vy_cmd, &w_cmd);

            controlRabbit.set_vx_cmd(vx_cmd);
            controlRabbit.set_vy_cmd(vy_cmd);
            controlRabbit.set_w_cmd(w_cmd);

            samplingStick = us_ticker_read();
        }

        /********************** ENCODER **********************/

        if (us_ticker_read() - samplingEncoder > ENC_MOTOR_SAMP_US_DEF)
        {
            controlRabbit.encoderMotorSamp();
            samplingEncoder = us_ticker_read();
        }

        /********************** PID & ODOM **********************/

        if (us_ticker_read() - samplingPID > PID_MOTOR_SAMP_US)
        {
            samplingPID = us_ticker_read();
            controlRabbit.pidMotorSamp();
        }

        if (us_ticker_read() - samplingOdom > SAMP_BASE_SPEED_US)
        {
            samplingOdom = us_ticker_read();
            controlRabbit.baseSpeed();
        }

        if (us_ticker_read() - samplingUpdPos > SAMP_UPD_POS_US)
        {
            controlRabbit.updatePosition();
            samplingUpdPos = us_ticker_read();
        }

        /********************** MOTOR **********************/

        if (us_ticker_read() - samplingMotor > MOTOR_SAMP_US)
        {
            controlRabbit.motorSamp();
            samplingMotor = us_ticker_read();
        }

        /********************** IK **********************/

        if (us_ticker_read() - samplingIK > SAMP_IK_US_DEF)
        {
            controlRabbit.base();
            samplingIK = us_ticker_read();
        }

        /********************** UPDATE ULTRASONIC **********************/
        // ultrasonicR.sensor();
        // ultrasonicL.sensor();
        // if (ultrasonicR.readable()){
        //     controlRabbit.setultrasonicR(ultR);

        // }
        // if (ultrasonicL.readable()){
        //     controlRabbit.setultrasonicL(ultL);

        // }




        /* MASTER: READ JOYSTICK FOR SPI */
        // if (us_ticker_read() - samplingStick > SAMP_STICK_US){
        //     stick.sendSPIStickState(&data_send_spi, &data_sampling_spi);

        //     // printf("READ SPI STATE: %d\n", data_send_spi);
        // }

        // /* MASTER: SEND SPI */
        // if (data_send_spi != 0 && us_ticker_read() - data_sampling_spi > SAMP_SPI_DATA){
        //     send_spi_inside = data_send_spi;

        //     state_send = unsigned(send_spi_inside);

        //     // printf("DEBUG STATE_SEND: %d\n", state_send);

        //     cs_m = 0;

        //     spi_m.write(state_send);
        //     resp = spi_m.write(STATE_IGN);

        //     cs_m = 1;

        //     data_sampling_spi = us_ticker_read();
        //     data_send_spi = 0;

        //     stick.set_send_spi(data_send_spi);
        //     stick.set_sampling_spi(data_sampling_spi);
        // }

        /** STICK **/
        // Directional Button

        // printf("Atas: %d\tKanan: %d\tBawah: %d\tKiri: %d\tSegitiga: %d\tLingkaran: %d\tKotak: %d\tSilang: %d\tL1: %d\tL2: %d\tR1: %d\tR2: %d\n", stick.getButtonUp(), stick.getButtonRight(), stick.getButtonDown(), stick.getButtonLeft(), stick.getSegitiga(), stick.getLingkaran(), stick.getKotak(), stick.getSilang(), stick.getL1(), stick.getL2(), stick.getR1(), stick.getR2());

        /** PRINT PWM ROADA **/
        if (us_ticker_read() - samplingIK > SAMP_IK_US_DEF)
        {
            // printf("ultL: %f\t", ultL);
            // printf("ultR: %f\n", ultR);

            // printf("ENC BR: %d\tBL: %d\tFR: %d\tFL: %d\n", encBR.getPulses(), encBL.getPulses(), encFR.getPulses(), encFL.getPulses());
            samplingIK = us_ticker_read();
        }

    }

    return 0;
}


