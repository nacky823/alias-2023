#ifndef MOTOR_HPP_
#define MOTOR_HPP_

#include "main.h"
#include "declare_extern.h"

#define FORWARD_L  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET)
#define BACKWARD_L HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET)
#define FORWARD_R  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET)
#define BACKWARD_R HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET)
#define COUNTER_PERIOD 8400.0

class Motor
{
#ifdef DEBUG_MODE
private:
    float translation_ratio_, rotation_ratio_;
    float sum_, excess_, reduced_translation_;
    float duty_l_, duty_r_;
    int16_t count_l_, count_r_;
#endif // DEBUG_MODE

public:
    Motor();
    void Init();
    void Drive(float, float);

#ifdef DEBUG_MODE
    void GetRatio(float &, float &);
    void GetLimitValues(float &, float &, float &);
    void GetDuty(float &, float &);
    void GetCount(int16_t &, int16_t &);
#endif // DEBUG_MODE

};

#endif // MOTOR_HPP_