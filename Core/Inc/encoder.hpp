#ifndef ENCODER_HPP_
#define ENCODER_HPP_

#include "main.h"

#define START_COUNT 32767
#define DISTANCE_PER_COUNT 0.007191 // tempolary [mm]
#define MACHINE_TREAD      127      // temp [mm]

#ifdef DEBUG_MODE
extern bool g_error_handler_tim8;
extern bool g_error_handler_tim4;
extern int16_t g_enc_cnt_l;
extern int16_t g_enc_cnt_r;
extern float g_distance_l;
extern float g_distance_r;
#endif // DEBUG_MODE

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