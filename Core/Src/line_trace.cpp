#include "line_trace.hpp"

LineTrace::LineTrace() : integral_error_(0) {}

void LineTrace::Init()
{
    integral_error_ = 0.0;
}

float LineTrace::PidControl(float p_gain, float i_gain, float d_gain)
{
    static float pre_error = line_sensor.LeftRightDifference();
    float error, differential_error, integral_error;
    float p, d, i;

    error = line_sensor.LeftRightDifference();
    differential_error = (error - pre_error) / INTERRUPT_PERIOD_TIM6_S;
    integral_error = (error + pre_error) / 2.0 * INTERRUPT_PERIOD_TIM6_S;
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

#ifdef DEBUG_MODE
float LineTrace::GetIntegralError()
{
    return integral_error_;
}
#endif // DEBUG_MODE