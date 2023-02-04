#ifndef VELOCITY_CONTROL_HPP_
#define VELOCITY_CONTROL_HPP_

#include "macro.h"
#include "encoder.hpp"

class VelocityControl
{
private:
    float integral_error_;

    void SetIntegralError(float);

    Encoder encoder;

public:
    VelocityControl();
    void Init();
    void PidControl(float, float, float, float);
    float GetIntegralError();

};

#endif // VELOCITY_CONTROL_HPP_