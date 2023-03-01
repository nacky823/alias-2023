#ifndef MOTOR_HPP_
#define MOTOR_HPP_

#include "main.h"

#define FORWARD_L  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET)
#define BACKWARD_L HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET)
#define FORWARD_R  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET)
#define BACKWARD_R HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET)
#define COUNTER_PERIOD 1049

class Motor
{
public:
    void Init();
    void Drive(float, float);

};

#endif // MOTOR_HPP_