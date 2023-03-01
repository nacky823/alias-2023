#ifndef VELOCITY_CONTROL_HPP_
#define VELOCITY_CONTROL_HPP_

#include "encoder.hpp"

class VelocityControl
{
private:
    float integral_error_;

    void SetIntegralError(float);
    float PidControl(float, float, float, float);

    Encoder *encoder_;

public:
    VelocityControl(Encoder *);
    float DeterminePidGain(float);
    void ResetIntegralError();

#ifdef DEBUG_MODE
    float GetIntegralError();
#endif // DEBUG_MODE

};

#endif // VELOCITY_CONTROL_HPP_