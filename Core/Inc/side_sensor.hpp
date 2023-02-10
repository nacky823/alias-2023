#ifndef SIDE_SENSOR_HPP_
#define SIDE_SENSOR_HPP_

#include "stm32f4xx_hal.h"
#include "declare_extern.h"

#define MAX_INTERRUPT_COUNT 250
#define BLACK_BLACK_COUNT 2
#define BLACK_WHITE_COUNT 1
#define WHITE_BLACK_COUNT 1
#define WHITE_WHITE_COUNT 0
#define IGNORE_COUNT      5
/* Digital side sensor : Black=1, White=0 */
#define IO_OUTSIDE_L HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_13)
#define IO_INSIDE_L  HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_14)
#define IO_OUTSIDE_R HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_2)
#define IO_INSIDE_R  HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_8)

class SideSensor
{
private:
    uint8_t read_state_flags_;
    uint8_t write_state_flags_;
    uint8_t exception_flags_;
    uint8_t master_count_;
    uint8_t corner_marker_count_;
    uint8_t cross_line_count_;

    void SensorUpdate();
    void NoiseTolerance();
    void CountUp();

public:
    SideSensor();
    void IgnoreJudgment();
    uint8_t GetGoalMarkerCount();
    uint8_t GetCornerMarkerCount();
    uint8_t GetCrossLineCount();

};

#endif // SIDE_SENSOR_HPP_