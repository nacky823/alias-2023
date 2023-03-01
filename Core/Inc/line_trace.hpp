#ifndef LINE_TRACE_HPP_
#define LINE_TRACE_HPP_

#include "line_sensor.hpp"

#define TARGET_VELOCITY_0 1.0
#define TARGET_VELOCITY_1 2.0

class LineTrace
{
private:
    float integral_error_;

    void SetIntegralError(float);
    float PidControl(float, float, float);

    LineSensor *line_sensor_;

public:
    LineTrace(LineSensor *);
    float DeterminePidGain(float);
    void ResetIntegralError();

#ifdef DEBUG_MODE
    float GetIntegralError();
#endif // DEBUG_MODE

};

#endif // LINE_TRACE_HPP_