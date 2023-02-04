#ifndef LINE_TRACE_HPP_
#define LINE_TRACE_HPP_

#include "line_sensor.hpp"

class LineTrace
{
private:
    float integral_error_;

    void SetIntegralError(float);

    LineSensor line_sensor;

public:
    LineTrace();
    void Init();
    float PidControl(float, float, float);

#ifdef DEBUG_MODE
    float GetIntegralError();
#endif // DEBUG_MODE

};

#endif // LINE_TRACE_HPP_