#ifndef PRINT_HPP_
#define PRINT_HPP_

#include "stm32f4xx_hal.h"
#include "flash.hpp"

#define LOG_LENGTH 6200
#define NUM_OF_BLANK 5

class Print
{
private:
    float distance_log_[LOG_LENGTH];
    float radian_log_[LOG_LENGTH];
    uint16_t various_log_[LOG_LENGTH];
    uint8_t accel_point_log[LOG_LENGTH];

    Flash *flash_;

    void HelloWorld();
    bool TestFlashWrite();
    bool TestIntFlash();
    void DistanceLog();
    void RadianLog();
    void VariousLog();
    void AccelPointLog();
    void Blank();

public:
    Print(Flash *);
    void Log();

};

#endif // PRINT_HPP_