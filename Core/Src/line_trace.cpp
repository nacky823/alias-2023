#include "line_trace.hpp"

LineTrace::LineTrace(LineSensor *line_sensor) : integral_error_(0)
{
    line_sensor_ = line_sensor;
}

float LineTrace::LineTraceOnly()
{
    float p_gain = LINE_ONLY_P_GAIN;
    float i_gain = LINE_ONLY_I_GAIN;
    float d_gain = LINE_ONLY_D_GAIN;

    return PidControl(p_gain, i_gain, d_gain);
}

float LineTrace::DeterminePidGain(float target_velocity)
{
    float p_gain, i_gain, d_gain;

    if(target_velocity < TARGET_VELOCITY_0)
    {
        p_gain = LINE_P_GAIN_0;
        i_gain = LINE_I_GAIN_0;
        d_gain = LINE_D_GAIN_0;
    }
    else if(target_velocity < TARGET_VELOCITY_1)
    {
        p_gain = LINE_P_GAIN_1;
        i_gain = LINE_I_GAIN_1;
        d_gain = LINE_D_GAIN_1;
    }
    else // Error handler
    {
        p_gain = 0;
        i_gain = 0;
        d_gain = 0;
    }

    return PidControl(p_gain, i_gain, d_gain);
}

float LineTrace::PidControl(float p_gain, float i_gain, float d_gain)
{
    static float pre_error = line_sensor_->LeftRightDifference();
    float error, differential_error, integral_error;
    float p, d, i;

    error = line_sensor_->LeftRightDifference();
    differential_error = (error - pre_error) / TIM6_PERIOD_S;
    integral_error = (error + pre_error) / 2.0 * TIM6_PERIOD_S;
    SetIntegralError(integral_error);

    p = p_gain * error;
    d = d_gain * differential_error;
    i = i_gain * integral_error_;

#ifdef DEBUG_MODE
    g_trace_p = p;
    g_trace_d = d;
    g_trace_i = i;
#endif // DEBUG_MODE

    pre_error = error;

    return p + d + i;
}

void LineTrace::SetIntegralError(float integral_error)
{
    integral_error_ += integral_error;
}

void LineTrace::ResetIntegralError()
{
    integral_error_ = 0.0;
}

#ifdef DEBUG_MODE
float LineTrace::GetIntegralError()
{
    return integral_error_;
}
#endif // DEBUG_MODE