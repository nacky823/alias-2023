#ifndef ENCODER_HPP_
#define ENCODER_HPP_

#include "stm32f4xx_hal.h"
#include "declare_extern.h"

#define START_COUNT 32767
#define DISTANCE_PER_COUNT 0.007191 // tempolary [mm]

class Encoder
{
private:
    float distance_;
    float distance_10mm_;
    float total_distance_;

public:
    Encoder();
    void Init();
    void Update();
    void Reset();
    void ResetDistance10mm();
    float GetDistance();
    float GetDistance10mm();
    float GetTotalDistance();

};

#endif // ENCODER_HPP_