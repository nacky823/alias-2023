#include "velocity_control.hpp"
#include "declare_extern.h"

VelocityControl::VelocityControl(Encoder *encoder) : integral_error_(0)
{
    encoder_ = encoder;
}

float VelocityControl::DeterminePidGain(float target_velocity)
{
    float p_gain, i_gain, d_gain;

    if(target_velocity < MAX_VELOCITY)
    {
        p_gain = VELOCITY_P_GAIN;
        i_gain = VELOCITY_I_GAIN;
        d_gain = VELOCITY_D_GAIN;
    }
    else // Error handler
    {
        p_gain = 0;
        i_gain = 0;
        d_gain = 0;
    }

    return PidControl(p_gain, i_gain, d_gain);
}

float VelocityControl::PidControl(float target, float p_gain, float i_gain, float d_gain)
{
    float current = encoder_->GetDistance() / TIM6_PERIOD; // [mm/ms]

    static float pre_filter = 0.0;
    float filter = pre_filter * (1.0 - VELOCITY_LPF_RATIO) + (current * VELOCITY_LPF_RATIO);
    pre_filter = filter;

    float error = target - filter;
    static float pre_error = error;

    float differential_error = (error - pre_error) / TIM6_PERIOD_S;
    float integral_error = (error + pre_error) / 2.0 * TIM6_PERIOD_S;
    SetIntegralError(integral_error);
    pre_error = error;

    float p = p_gain * error;
    float d = d_gain * differential_error;
    float i = i_gain * integral_error_;

#ifdef DEBUG_MODE
    g_target_velocity  = target;
    g_current_velocity = current;
    g_filter_velocity  = filter;
    g_velocity_error   = error;
    g_velocity_p = p;       g_velocity_d = d;       g_velocity_i = i;
    g_velocity_p_gain = p;  g_velocity_d_gain = p;  g_velocity_i_gain = p;
#endif // DEBUG_MODE

    return p + d + i;
}

void VelocityControl::SetIntegralError(float integral_error)
{
    integral_error_ += integral_error;
}

void VelocityControl::ResetIntegralError()
{
    integral_error_ = 0.0;
}

#ifdef DEBUG_MODE
float VelocityControl::GetIntegralError()
{
    return integral_error_;
}
#endif // DEBUG_MODE