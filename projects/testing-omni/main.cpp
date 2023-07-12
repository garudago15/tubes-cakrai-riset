#include "mbed.h"
#include "../../libs/Configs/Constants.h"
#include "../../libs/JoystickPS3/JoystickPS3.h"
#include "../libs/Control4Roda/Control4Omni.h"

/* INITIALIZE SERIAL USING PRINTF */
static BufferedSerial serial_port(USBTX, USBRX, 115200);
FileHandle *mbed::mbed_override_console(int fd){
    return &serial_port;
}

/* INITIALIZE PS3 JOYSTICK */
JoystickPS3 ps3(PS3_TX, PS3_RX);

/* INITIALIZE OMNIWHEEL */
/* Berikut adalah cara pake omniwheel:
1. untuk ubah forward-backward speed pake omni.set_vy_cmd(float dlm m/s (depan positif))
2. untuk ubah left-right speed pake omni.set_vx_cmd(float dlm m/s (kanan positif))
3. untuk ubah rotasi pake omni.set_w_cmd(float dlm m/s (berlawanan arah jarum jam positif))
4. untuk rem pake omni.forceBrakeSync()
note: asumsi menggunakan 3 encoder posisi (utk odometri)
note: asumsi full manual operation */
Motor *FL_Motor(FL_PWM, FL_FWD, FL_REV);
Motor *FR_Motor(FR_PWM, FR_FWD, FR_REV);
Motor *BL_Motor(BL_PWM, BL_FWD, BL_REV);
Motor *BR_Motor(BR_PWM, BR_FWD, BR_REV);
encoderKRAI *encFL(FL_CHA, FL_CHB, WHEEL_PPR, Encoding::X4_ENCODING);
encoderKRAI *encFR(FR_CHA, FR_CHB, WHEEL_PPR, Encoding::X4_ENCODING);
encoderKRAI *encBR(BR_CHA, BR_CHB, WHEEL_PPR, Encoding::X4_ENCODING);
encoderKRAI *encBL(BL_CHA, BL_CHB, WHEEL_PPR, Encoding::X4_ENCODING);
pidLo *FLPid(KP_WHEEL_PID,KI_WHEEL_PID,0,TIME_SAMPLING, 1, 0, 1000, 1000); //constants: kp, ki, kd
pidLo *FRPid(KP_WHEEL_PID,KI_WHEEL_PID,0,TIME_SAMPLING, 1, 0, 1000, 1000);
pidLo *BRPid(KP_WHEEL_PID,KI_WHEEL_PID,0,TIME_SAMPLING, 1, 0, 1000, 1000);
pidLo *BLPid(KP_WHEEL_PID,KI_WHEEL_PID,0,TIME_SAMPLING, 1, 0, 1000, 1000);
SMC *FLSMC(KP_WHEEL_SMC, KS_WHEEL_SMC, EPS_WHEEL_SMC, 1, TIME_SAMPLING, SMC::KECEPATAN); //constants: kp, ksigma, epsilon, dan beta(tidak terpakai)
SMC *FRSMC(KP_WHEEL_SMC, KS_WHEEL_SMC, EPS_WHEEL_SMC, 1, TIME_SAMPLING, SMC::KECEPATAN);
SMC *BRSMC(KP_WHEEL_SMC, KS_WHEEL_SMC, EPS_WHEEL_SMC, 1, TIME_SAMPLING, SMC::KECEPATAN);
SMC *BLSMC(KP_WHEEL_SMC, KS_WHEEL_SMC, EPS_WHEEL_SMC, 1, TIME_SAMPLING, SMC::KECEPATAN);
//ControlMotor digunakan untuk menggabung hasil pid dan smc serta memberi nilai kp_pid, kd_pid, dan kp_smc sebagai fungsi kecepatan (dengan k1 sebagai minimal dan k2 sebagai maksimal) 
ControlMotor *control_FL_motor(FLPid, FLSMC, MAX_WHEEL_SPEED, KP_WHEEL_PID_MIN, KP_WHEEL_PID_MAX, KD_WHEEL_PID_MIN, KD_WHEEL_PID_MAX, KP_WHEEL_SMC_MIN, KP_WHEEL_SMC_MAX); //constants: kp_pid_1, kp_pid_2, kd_pid_1, kd_pid_2, kp_smc_1, kp_smc_2
ControlMotor *control_FR_motor(FRPid, FRSMC, MAX_WHEEL_SPEED, KP_WHEEL_PID_MIN, KP_WHEEL_PID_MAX, KD_WHEEL_PID_MIN, KD_WHEEL_PID_MAX, KP_WHEEL_SMC_MIN, KP_WHEEL_SMC_MAX);
ControlMotor *control_BR_motor(BRPid, BRSMC, MAX_WHEEL_SPEED, KP_WHEEL_PID_MIN, KP_WHEEL_PID_MAX, KD_WHEEL_PID_MIN, KD_WHEEL_PID_MAX, KP_WHEEL_SMC_MIN, KP_WHEEL_SMC_MAX);
ControlMotor *control_BL_motor(BLPid, BLSMC, MAX_WHEEL_SPEED, KP_WHEEL_PID_MIN, KP_WHEEL_PID_MAX, KD_WHEEL_PID_MIN, KD_WHEEL_PID_MAX, KP_WHEEL_SMC_MIN, KP_WHEEL_SMC_MAX);
//encoder eksternal (encoderHAL) kyknya perlu research dulu karena pake namespace mbed (dan melibatkan timer interrupt). Untuk sekarang pemahaman saya sebagai berikut:
// TIM_TypeDef *encL_TIM=encL_TIM; //for reference see www.st.com/st-web-ui/static/active/en/resource/technical/document/datasheet/DM00102166.pdf 
// TIM_TypeDef *encR_TIM=encR_TIM; //this is also useful https://github.com/STMicroelectronics/STM32CubeF4/blob/master/Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_tim.h
// TIM_TypeDef *encAux_TIM=encAux_TIM; //and this for a similar problem https://os.mbed.com/forum/platform-34-ST-Nucleo-F401RE-community/topic/4963/ 
encoderHAL encL(encL_TIM);
encoderHAL encR(encR_TIM);
encoderHAL encAux(encAux_TIM);
HAL_TIM_Encoder_MspInit(encL.getTimer());
HAL_TIM_Encoder_MspInit(encR.getTimer());
HAL_TIM_Encoder_MspInit(encAux.getTimer());
odom3enc *odom(*encL, *encR, *encAux);
pidLo *vxPid(KP_ROBOT_PID_v,KI_ROBOT_PID_v,0,TIME_SAMPLING, 1, 0, 1000, 1000);
pidLo *vyPid(KP_ROBOT_PID_v,KI_ROBOT_PID_v,0,TIME_SAMPLING, 1, 0, 1000, 1000);
pidLo *wPid(KP_ROBOT_PID_w,KI_ROBOT_PID_w,0,TIME_SAMPLING, 1, 0, 1000, 1000);
//3 baris di bawah ini tidak akan dipakai karena omniWheel dikendalikan manual
StanleyPursuit *line;
pidLo *pid(0,0,0,TIME_SAMPLING, 1, 0, 1000, 1000);
pidLo *pid2(0,0,0,TIME_SAMPLING, 1, 0, 1000, 1000);
//define omniWheel
Control4Omni omni(FL_motor, FR_motor, BR_motor, BL_motor, encFL, encFR, encBR, encBL, control_FL_motor, control_FR_motor, control_BR_motor, control_BL_motor, odom, vxPid, vyPid, wPid, line, pid, pid2); //*line, *pid, dan *pid2 digunakan utk automatic positioning

/* INITIALIZE TIMERS*/
int timer1 = us_ticker_read();
int timer2 = us_ticker_read();
int now = us_ticker_read();

int main(){
    ps3.setup();
    omni.encoderMotorSamp(); //baca data awal encoder omniWheel
    while (true){
        // Pengolahan dan Update data PS3
        ps3.olah_data();
        ps3.baca_data();
        if(ps3.getKotak()){
            omni.forceBrakeSync(); //hard brake sebelum nembak agar robot tidak gerak saat nembak
        }
        if(ps3.getR2()){
            if(ps3.getButtonUp()){ // Maju Boosted
                omni.set_vy_cmd(MAX_ROBOT_SPEED);
            } else if(ps3.getButtonDown()){ // Mundur Boosted
                omni.set_vy_cmd(-1 * MAX_ROBOT_SPEED);
            } else{ //Soft Brake
                omni.set_vy_cmd(0);
            }
            if(ps3.getButtonRight()){ // Kanan Boosted
                omni.set_vx_cmd(MAX_ROBOT_SPEED);
            } else if(ps3.getButtonLeft()){ // Kiri Boosted
                omni.set_vx_cmd(-1 * MAX_ROBOT_SPEED);
            } else{ //Soft Brake
                omni.set_vx_cmd(0);
            }
            if(ps3.getL1()){ //Mutar berlawanan arah jarum jam Boosted
                omni.set_w_cmd(MAX_ROBOT_SPIN);
            } else if(ps3.getR1()){ //Mutar searah jarum jam Boosted
                omni.set_w_cmd(-1 * MAX_ROBOT_SPIN);
            } else{ //Soft Brake
                omni.set_w_cmd(0);
            }
        }else{
            if(ps3.getButtonUp()){ // Maju
                omni.set_vy_cmd(MAX_ROBOT_SPEED / 2);
            } else if(ps3.getButtonDown()){ // Mundur
                omni.set_vy_cmd(-1 * MAX_ROBOT_SPEED / 2);
            } else{ //Soft Brake
                omni.set_vy_cmd(0);
            }
            if(ps3.getButtonRight()){ // Kanan
                omni.set_vx_cmd(MAX_ROBOT_SPEED / 2);
            } else if(ps3.getButtonLeft()){ // Kiri
                omni.set_vx_cmd(-1 * MAX_ROBOT_SPEED / 2);
            } else{ //Soft Brake
                omni.set_vx_cmd(0);
            }
            if(ps3.getL1()){ //Mutar berlawanan arah jarum jam
                omni.set_w_cmd(MAX_ROBOT_SPIN / 2);
            } else if(ps3.getR1()){ //Mutar searah jarum jam
                omni.set_w_cmd(-1 * MAX_ROBOT_SPIN / 2);
            } else{ //Soft Brake
                omni.set_w_cmd(0);
            }
        }

        // PID
        now = us_ticker_read();
        if(now - timer1 > TIME_SAMPLING){
            //omni sampling
            omni.encoderMotorSamp(); //baca encoder (current speed) tiap motor
            omni.baseSpeed(); //update odometri
            omni.base(); //update target speed tiap motor berdasarkan vx, vy, dan w
            omni.pidMotorSamp(); //update PID berdasarkan target dan current speed tiap motor
            omni.motorSamp(); //update motor

            timer1 = now;
        }
    }
}