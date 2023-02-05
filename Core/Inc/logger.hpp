#ifndef LOGGER_HPP_
#define LOGGER_HPP_

#include "encoder.hpp"
#include "side_sensor.hpp"
#include "iim_42652.hpp"

#define MAX_LOG_INDEX 6200
#define FORMAL_10MM   10.0

class Logger
{
private:
    /* Data at 10mm intervals. */
    uint16_t various_data_[MAX_LOG_INDEX]; // 12.4 [kByte]
    int16_t gyro_data_yaw_[MAX_LOG_INDEX]; // 12.4 [kByte]
    float excess_distance_[MAX_LOG_INDEX]; // 24.8 [kByte]

    Encoder encoder;
    SideSensor side_sensor;
    Iim42652 iim_42652;

public:
    Logger();
    void Logging(uint8_t);

};

#endif // LOGGER_HPP_