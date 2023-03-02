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
    float GetIntegralError();

};

#endif // VELOCITY_CONTROL_HPP_