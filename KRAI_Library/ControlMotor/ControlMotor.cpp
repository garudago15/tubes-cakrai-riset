#include "ControlMotor.h"

ControlMotor::ControlMotor(PID* address_pid, SMC * address_smc, float v_batas, float kp_pid_1, float kp_pid_2,  float kd_pid_1, float kd_pid_2, float kp_smc_1, float kp_smc_2): _address_pid(address_pid), _address_smc(address_smc), _kp_pid_1(kp_pid_1), _kp_pid_2(kp_pid_2), _kd_pid_1(kd_pid_1), _kd_pid_2(kd_pid_2), _kp_smc_1(kp_smc_1), _kp_smc_2(kp_smc_2), _v_batas(v_batas)
{
    (*address_pid).setKp(this->_kp_pid_1);
    (*address_pid).setKd(this->_kd_pid_1);
    (*address_smc).setKp(this->_kp_smc_1);
}

ControlMotor::ControlMotor(PIDAaronBerk* address_pid_aaron, SMC * address_smc, float v_batas, float kc_pid_unsteady, float kc_pid_steady, float kp_smc_1, float kp_smc_2): _address_pid_aaron(address_pid_aaron), _address_smc(address_smc), _kc_pid_unsteady(kc_pid_unsteady), _kc_pid_steady(kc_pid_steady), _kp_smc_1(kp_smc_1), _kp_smc_2(kp_smc_2), _v_batas(v_batas)
{
    (*address_pid_aaron).setKc(this->_kc_pid_unsteady);
    (*address_smc).setKp(this->_kp_smc_1);
}

ControlMotor::ControlMotor(pidLo* address_pidLo, SMC * address_smc, float v_batas, float kp_pid_1, float kp_pid_2, float kd_pid_1, float kd_pid_2, float kp_smc_1, float kp_smc_2): _address_pidLo(address_pidLo), _address_smc(address_smc), _kp_pid_1(kp_pid_1), _kp_pid_2(kp_pid_2), _kd_pid_1(kd_pid_1), _kd_pid_2(kd_pid_2), _kp_smc_1(kp_smc_1), _kp_smc_2(kp_smc_2), _v_batas(v_batas)
{
    (*address_pidLo).setKp(this->_kp_pid_1);
    (*address_pidLo).setKd(this->_kd_pid_1);
    (*address_smc).setKp(this->_kp_smc_1);
}

// Fungsi Signum, bentuk seperti S, digunakan untuk
float ControlMotor::sgn(float in){
    return (in / (fabs(in) + 0.001));
}

// Untuk kecepatan beda, PID beda
// rumus_geser diperuntukkan agar mencari konstanta PID yang sesuai untuk kecepatan tertentu
float ControlMotor::rumus_geser(float in, float vb, float k1, float k2){
    return (this->sgn(fabs(in) - vb)* (k2-k1) + k2 + k1 )/2;
}

// Set komponen PID dan SMC berdasarkan state dan feedback
// float ControlMotor::createpwm_old(float setpoint, float feedback, float max){
float ControlMotor::createpwm(float setpoint, float feedback, float max){
    if (setpoint == 0)
    {
        (*_address_pid).reset();
        (*_address_smc).reset();
    }

    // (*_address_pid).setKp(rumus_geser(setpoint - feedback, _v_batas, _kp_pid_1, _kp_pid_2));
    // (*_address_pid).setKd(rumus_geser(setpoint - feedback, _v_batas, _kd_pid_1, _kd_pid_2));
    // (*_address_smc).setKp(rumus_geser(setpoint - feedback, _v_batas, _kp_smc_1, _kp_smc_2));

    float temp = (*_address_pid).createpwm(setpoint, feedback, max) + (*_address_smc).createpwm(setpoint, feedback, max);

    // printf("PID: %f, SMC: %f\n", (*_address_pid).createpwm(setpoint, feedback, max), (*_address_smc).createpwm(setpoint, feedback, max));

    if (fabs(temp) > fabs(max)){
        temp = fabs(temp)*max/temp;
    }
    return temp;
}


// float ControlMotor::createpwm(float setpoint, float feedback, float max){
float ControlMotor::newcreatepwm(float setpoint, float feedback, float max){
    if (setpoint == 0)
    {
        (*_address_pid_aaron).reset();
        (*_address_smc).reset();
    }

    (*_address_smc).setKp(rumus_geser(setpoint - feedback, _v_batas, _kp_smc_1, _kp_smc_2));
    (*_address_pid_aaron).setKc(rumus_geser(setpoint - feedback, _v_batas, _kc_pid_steady, _kc_pid_unsteady));

    // float temp = (*_address_pid_aaron).advancepwm(setpoint / 2, feedback / 2, max / 2) + (*_address_smc).createpwm(setpoint / 2, feedback / 2, max / 2);
    // float temp = (*_address_pid_aaron).advancepwm(setpoint * 0.75, feedback * 0.75, max * 0.75) + (*_address_smc).createpwm(setpoint * 0.25, feedback * 0.25, max * 0.25);
    float temp = (*_address_smc).createpwm(setpoint, feedback, max);
    // float temp = (*_address_pid_aaron).advancepwm(setpoint, feedback, max);
    // float temp = (*_address_pid_aaron).advancepwm(setpoint, feedback, max) + (*_address_smc).createpwm(setpoint, feedback, max);

    // printf("PID: %f, SMC: %f\n", (*_address_pid_aaron).createpwm(setpoint, feedback, max), (*_address_smc).createpwm(setpoint, feedback, max));

    return temp;
}

float ControlMotor::createpwmPidLo(float setpoint, float feedback, float max){
    if (setpoint == 0)
    {
        (*_address_pidLo).reset();
        (*_address_smc).reset();
    }


    float temp = (*_address_pidLo).createpwm(setpoint, feedback, max) + (*_address_smc).createpwm(setpoint, feedback, max);
    // float temp = ((*_address_pidLo).createpwm(setpoint, feedback, max) + (*_address_smc).createpwm(setpoint, feedback, max))/2.0f;

    
    if (fabs(temp) > fabs(max)){
        temp = fabs(temp)*max/temp;
    }
    return temp;
}

void ControlMotor::reset(){
    (*_address_pidLo).reset();
    (*_address_smc).reset();
}