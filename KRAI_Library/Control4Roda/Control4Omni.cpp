#include "Control4Omni.h"
#include <math.h>

Control4Omni::Control4Omni(Motor *FL_motor, Motor *FR_motor, Motor *BR_motor, Motor *BL_motor, encoderKRAI *encFL, encoderKRAI *encFR, encoderKRAI *encBR, encoderKRAI *encBL,
                           ControlMotor *control_FL_motor, ControlMotor *control_FR_motor, ControlMotor *control_BR_motor, ControlMotor *control_BL_motor) : Control4Roda(FL_motor, FR_motor, BR_motor, BL_motor, encFL, encFR, encBR, encBL, control_FL_motor, control_FR_motor, control_BR_motor, control_BL_motor)
{
    // this->line->setError(ERROR_THRESHOLD);
}

void Control4Omni::encoderMotorSamp()
{
    this->baseSpeed();

    uint32_t currBLticker = us_ticker_read();
    this->v_BL_curr = (float)this->encBL->getPulses() * 2 * PI * WHEEL_RAD * S_TO_US / (ENC_MOTOR_PULSE * (currBLticker - this->prevBLticker));
    this->prevBLticker = currBLticker;
    this->encBL->reset();

    uint32_t currFLticker = us_ticker_read();
    this->v_FL_curr = (float)this->encFL->getPulses() * 2 * PI * WHEEL_RAD * S_TO_US / (ENC_MOTOR_PULSE * (currFLticker - this->prevFLticker));
    this->prevFLticker = currFLticker;
    this->encFL->reset();

    uint32_t currBRticker = us_ticker_read();
    this->v_BR_curr = (float)this->encBR->getPulses() * 2 * PI * WHEEL_RAD * S_TO_US / (ENC_MOTOR_PULSE * (currBRticker - this->prevBRticker));
    this->prevBRticker = currBRticker;
    this->encBR->reset();

    uint32_t currFRticker = us_ticker_read();
    this->v_FR_curr = (float)this->encFR->getPulses() * 2 * PI * WHEEL_RAD * S_TO_US / (ENC_MOTOR_PULSE * (currFRticker - this->prevFRticker));
    this->prevFRticker = currFRticker;
    this->encFR->reset();

    // this->v_FL_curr = (float)this->encFL->getPulses()*2*PI*WHEEL_RAD*S_TO_US/(ENC_MOTOR_PULSE*ENC_MOTOR_SAMP_US_DEF);
    // this->v_FR_curr = (float)this->encFR->getPulses()*2*PI*WHEEL_RAD*S_TO_US/(ENC_MOTOR_PULSE*ENC_MOTOR_SAMP_US_DEF);
    // this->v_BR_curr = (float)this->encBR->getPulses()*2*PI*WHEEL_RAD*S_TO_US/(ENC_MOTOR_PULSE*ENC_MOTOR_SAMP_US_DEF);
    // this->v_BL_curr = (float)this->encBL->getPulses()*2*PI*WHEEL_RAD*S_TO_US/(ENC_MOTOR_PULSE*ENC_MOTOR_SAMP_US_DEF);

    // printf("v_FL_curr: %f\tv_FR_curr: %f\tv_BR_curr: %f\tv_BL_curr: %f\n", this->v_FL_curr, this->v_FR_curr, this->v_BR_curr, this->v_BL_curr);
    // printf("%f\t%f\t%f\t%f\n", this->v_FL_curr, this->v_FR_curr, this->v_BR_curr, this->v_BL_curr);
    /* reset nilai encoder */
}

void Control4Omni::baseSpeed()
{
    this->updatePosition();

    // printf("%f %f %f\t\t%d %f %f\n", this->odom->position.x, this->odom->position.y, this->odom->position.teta, curr_dest_cout, arr_x_offline_atas_1[curr_dest_cout], arr_y_offline_atas_1[curr_dest_cout]);
}

void Control4Omni::base()
{

    // Robot jalannya lurus, ga perlu koreksi pake vc vy w PID
    this->vy_motor = this->vy_cmd;
    this->vx_motor = this->vx_cmd;
    this->w_motor = this->w_cmd;

    if (fabs(this->vy_motor - this->vy_last) > (SAMP_IK_US / 1000000.0) * MAX_ACCEL_Y)
    {
        if (this->vy_last > this->vy_motor)
        {
            this->vy_motor = this->vy_last - MAX_ACCEL_Y * (SAMP_IK_US / 1000000.0);
        }
        else
        {
            this->vy_motor = this->vy_last + MAX_ACCEL_Y * (SAMP_IK_US / 1000000.0);
        }
    }
    if (fabs(this->vx_motor - this->vx_last) > (SAMP_IK_US / 1000000.0) * MAX_ACCEL_X)
    {
        if (this->vx_last > this->vx_motor)
        {
            this->vx_motor = this->vx_last - MAX_ACCEL_X * (SAMP_IK_US / 1000000.0);
        }
        else
        {
            this->vx_motor = this->vx_last + MAX_ACCEL_X * (SAMP_IK_US / 1000000.0);
        }
    }
    if (fabs(this->w_motor - this->w_last) > (SAMP_IK_US / 1000000.0) * MAX_ACCEL_W)
    {
        if (this->w_last > this->w_motor)
        {
            this->w_motor = this->w_last - MAX_ACCEL_W * (SAMP_IK_US / 1000000.0);
        }
        else
        {
            this->w_motor = this->w_last + MAX_ACCEL_W * (SAMP_IK_US / 1000000.0);
        }
    }
    this->vy_last = this->vy_motor;
    this->vx_last = this->vx_motor;

    float vx_motor_input = this->vx_motor * COS45;
    float vy_motor_input = this->vy_motor * COS45;
    float w_motor_input = this->w_motor;

    this->FL_target_speed = vy_motor_input + vx_motor_input - w_motor_input * R_BASE;
    this->FR_target_speed = -vy_motor_input + vx_motor_input - w_motor_input * R_BASE;
    this->BR_target_speed = -vy_motor_input - vx_motor_input - w_motor_input * R_BASE;
    this->BL_target_speed = vy_motor_input - vx_motor_input - w_motor_input * R_BASE;

    // printf("FL target: %f\tFR target: %f\tBR target: %f\tBL target: %f\n", this->FL_target_speed, this->FR_target_speed, this->BR_target_speed, this->BL_target_speed);
}

void Control4Omni::basePidLo()
{

    // Robot jalannya lurus, ga perlu koreksi pake vc vy w PID
    this->vy_motor = this->vy_cmd;
    this->vx_motor = this->vx_cmd;
    this->w_motor = this->w_cmd;

    
    this->vy_last = this->vy_motor;
    this->vx_last = this->vx_motor;

    float vx_motor_input = this->vx_motor / COS45;
    float vy_motor_input = this->vy_motor / COS45;
    float w_motor_input = this->w_motor;

    this->FL_target_speed = vy_motor_input + vx_motor_input - w_motor_input * R_BASE;
    this->FR_target_speed = -vy_motor_input + vx_motor_input - w_motor_input * R_BASE;
    this->BR_target_speed = -vy_motor_input - vx_motor_input - w_motor_input * R_BASE;
    this->BL_target_speed = vy_motor_input - vx_motor_input - w_motor_input * R_BASE;

    // printf("FL target: %f\tFR target: %f\tBR target: %f\tBL target: %f\n", this->FL_target_speed, this->FR_target_speed, this->BR_target_speed, this->BL_target_speed);
}
