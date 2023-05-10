#include "imu.hpp"
#include "declare_extern.h"

Imu::Imu(Iim42652 *iim_42652) : rad_z_(0)
                              , rad_stack_z_(0)
                              , clear_integral_error_(0)
{
    iim_42652_ = iim_42652;
}

void Imu::Update()
{
    int16_t raw_gyro_z_l = iim_42652_->GyroZLeft();
    int16_t raw_gyro_z_r = iim_42652_->GyroZRight();
    float avg_raw_gyro_z = (raw_gyro_z_l + raw_gyro_z_r) / 2.0;
    float rad_z = avg_raw_gyro_z * CONST_RAD_CALC * TIM6_PERIOD_S;

    /* low pass filter */
    static float pre_filter = 0;
    float filter = pre_filter * (1.0 - RAD_LPF_RATIO) + (rad_z * RAD_LPF_RATIO);
    pre_filter = filter;

    rad_z_ = filter;
    rad_stack_z_ += filter;

#ifdef DEBUG_MODE
    g_rad_z = rad_z;
    g_rad_stack_z = rad_stack_z_;
#endif // DEBUG_MODE
}

void Imu::ClearRadStackZ()
{
    rad_stack_z_ = 0;
}

float Imu::GetRadStackZ()
{
    return rad_stack_z_;
}

float Imu::GetRadZ()
{
    return rad_z_;
}

float Imu::PidControl(float error)
{
    static float pre_error = error;
    static float integral_error = 0;
    float differential_error;
    float p, i, d;
    
    integral_error += (error + pre_error) / 2.0 * TIM6_PERIOD_S;
    differential_error = (error - pre_error) / TIM6_PERIOD_S;

    pre_error = error;

    if(clear_integral_error_) integral_error = 0;

    p = GYRO_P_GAIN * error;
    i = GYRO_I_GAIN * integral_error;
    d = GYRO_D_GAIN * differential_error;

    return p + i + d;
}

#ifdef DEBUG_MODE
void Imu::CalibrationStackRadian()
{
    int16_t yaw_l = iim_42652_->GyroZLeft();
    int16_t yaw_r = iim_42652_->GyroZRight();
    float yaw_avg = (yaw_l + yaw_r) / 2.0;
    float radian = yaw_avg * CONST_RAD_CALC * TIM6_PERIOD_S;

    rad_stack_z_ += radian;
    g_calib_rad_stack = rad_stack_z_;
}

void Imu::Calibration()
{
    for(uint32_t i = 0; i < NUM_OF_SAMPLE_CALIB; i++)
    {
        CalibrationStackRadian();
        HAL_Delay(1);
    }
}
#endif // DEBUG_MODE