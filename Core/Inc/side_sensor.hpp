#ifndef SIDE_SENSOR_HPP_
#define SIDE_SENSOR_HPP_

#include "stm32f4xx_hal.h"
#include "declare_extern.h"

#define MAX_INTERRUPT_COUNT 250 // tim6[ms]
#define BLACK_BLACK_COUNT   2   // tim6[ms]
#define BLACK_WHITE_COUNT   1   // tim6[ms]
#define WHITE_BLACK_COUNT   1   // tim6[ms]
#define WHITE_WHITE_COUNT   0   // tim6[ms]
#define IGNORE_COUNT        5   // tim6[ms]
/* Digital side sensor : Black=1, White=0 */
#define IO_OUTSIDE_L HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_13)
#define IO_INSIDE_L  HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_14)
#define IO_OUTSIDE_R HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_2)
#define IO_INSIDE_R  HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_8)

class SideSensor
{
private:
    uint8_t read_state_flags_;
    /*  0000 0000
     *  |||| ||||
     *  |||| now_marker_state
     *  pre_marker_state
     */
    uint8_t write_state_flags_;
    /*  0000 0000
     *  |||  ||||
     *  |||  |||cross_flag
     *  |||  ||corner_flag
     *  |||  |goal_flag
     *  |||  black_flag
     *  ||cross_reach
     *  |corner_reach
     *  goal_reach
     */
    uint8_t exception_flags_;
    /*  0000 0000
     *  |||| || |
     *  |||| || ignore_flag
     *  |||| noise_count
     *  before_noise_state
     */
    uint8_t master_count_;
    uint8_t goal_marker_count_;
    uint8_t corner_marker_count_;
    uint8_t cross_line_count_;

    void UpdateState();
    void NoiseTolerance();
    void ConfirmState();
    void CountUp();

public:
    SideSensor();
    void Update();
    uint8_t GetGoalMarkerCount();
    uint8_t GetCornerMarkerCount();
    uint8_t GetCrossLineCount();

#ifdef DEBUG_MODE
    void Monitor();
#endif // DEBUG_MODE

};

#endif // SIDE_SENSOR_HPP_