#ifndef _CONTROLMOTOR_H_
#define _CONTROLMOTOR_H_

#include "mbed.h"
#include "../pidLo/pidLo.h"
#include "../PID_KRAI/PID_KRAI.h"
#include "../PIDAaronBerk/PIDAaronBerk.h"
#include "../SMC_KRAI/SMC_KRAI.h"

class ControlMotor {
    private:
        PID *_address_pid;
        PIDAaronBerk *_address_pid_aaron;
        SMC *_address_smc;
        uint32_t _total_state;

        float _kp_pid_1;
        float _kp_pid_2;

        float _kd_pid_1;
        float _kd_pid_2;

        float _kp_smc_1;
        float _kp_smc_2;

        float _kc_pid_unsteady;
        float _kc_pid_steady;

        float _v_batas;

        uint32_t _state_curr;
    
    public:
        /** Create ControlMotor Interface
         * @param SMC
         * @param PID
         * 
         */
        ControlMotor(PID* address_pid, SMC* address_smc, float v_batas, float kp_pid_1, float kp_pid_2, float kd_pid_1, float kd_pid_2, float kp_smc_1, float kp_smc_2);
        ControlMotor(PIDAaronBerk* address_pid_aaron, SMC* address_smc, float v_batas, float kc_pid_unsteady, float kc_pid_steady, float kp_smc_1, float kp_smc_2);
        float sgn(float in);
        float rumus_geser(float in, float vb, float k1, float k2);
        float createpwm(float setpoint, float feedback, float max);
        float newcreatepwm(float setpoint, float feedback, float max);
};

#endif