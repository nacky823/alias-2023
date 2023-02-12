#ifndef ENCODER_HPP_
#define ENCODER_HPP_

#include "stm32f4xx_hal.h"
#include "declare_extern.h"

#define START_COUNT 32767
#define DISTANCE_PER_COUNT 0.007191 // tempolary [mm]
#define MACHINE_TREAD 404 //temp [mm]

class Encoder
{
private:
    float distance_;
    float distance_stack_;
    float distance_difference_;

public:
    Encoder();
    void Init();
    void Update();
    void Reset();
    void ResetDistanceStack();
    float GetDistance();
    float GetDistanceStack();
    float AngularVelocity();

};

#endif // ENCODER_HPP_