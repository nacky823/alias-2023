#ifndef LED_HPP_
#define LED_HPP_

#include "stm32f4xx_hal.h"

#define R_ON  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET)
#define R_OFF HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET)
#define G_ON  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET)
#define G_OFF HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET)
#define B_ON  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET)
#define B_OFF HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET)
#define BLINK_INTERVAL_MS   400
#define RAINBOW_INTERVAL_MS 300

class Led
{
public:
    void ColorOrder(char);
    void Blink(uint8_t, char, char);
    void Rainbow(uint8_t);

};

#endif // LED_HPP_