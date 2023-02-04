#ifndef ROTARY_SWITCH_HPP_
#define ROTARY_SWITCH_HPP_

#include "stm32f4xx_hal.h"

#define PIN_1 HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_12)
#define PIN_2 HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_2)
#define PIN_4 HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_1)
#define PIN_8 HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0)

class RotarySwitch
{
public:
    uint8_t State();

};

#endif // ROTARY_SWITCH_HPP_