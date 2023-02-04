#ifndef ENCODER_HPP_
#define ENCODER_HPP_

#include "stm32f4xx_hal.h"
#include "declare_extern.h"

#define START_COUNT 32767
#define DISTANCE_PER_COUNT 0.007191 // tempolary [mm]

class Encoder
{
private:
    int16_t count_l_, count_r_;
    float distance_;
    float total_distance_;

public:
    Encoder();
    void Init();
    void ResetCountDistance();
    void UpdateCountDistance();
    void AddTotalDistance();
    void GetCount(int16_t &, int16_t &);
    float GetDistance();
    float GetTotalDistance();

};

#endif // ENCODER_HPP_