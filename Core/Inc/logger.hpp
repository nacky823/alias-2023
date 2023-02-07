#ifndef LOGGER_HPP_
#define LOGGER_HPP_

#include "encoder.hpp"
#include "side_sensor.hpp"
#include "iim_42652.hpp"

#define LOG_MAX_CNT_10MM 6200
#define FORMAL_10MM      10.0
#define NORMAL_FUCOR_DPS 0.0610370 // 2000 / 32768

class Logger
{
private:
    /* Data at 10mm intervals. */
    uint16_t various_data_[LOG_MAX_CNT_10MM]; // 12.4 [kByte]
    uint16_t time_10mm_ms_[LOG_MAX_CNT_10MM]; // 12.4 [kByte]
    int16_t gyro_data_yaw_[LOG_MAX_CNT_10MM]; // 12.4 [kByte]
    float excess_distance_[LOG_MAX_CNT_10MM]; // 24.8 [kByte]

    Encoder encoder;
    SideSensor side_sensor;
    Iim42652 iim_42652;
    Led led;

public:
    Logger();
    void Logging(uint8_t);
    void StoreLog();
    float FindTargetVelocity();

};

#endif // LOGGER_HPP_