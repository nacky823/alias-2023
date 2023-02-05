#ifndef LOGGER_HPP_
#define LOGGER_HPP_

#define MAX_LOG_INDEX 6200

class Logger
{
private:
    /* Data at 10mm intervals. */
    uint16_t various_data_[MAX_LOG_INDEX]; // 12.4 [kByte]
    int16_t gyro_data_yaw_[MAX_LOG_INDEX]; // 12.4 [kByte]
    float excess_distance_[MAX_LOG_INDEX]; // 24.8 [kByte]

public:
    Logger();

};

#endif // LOGGER_HPP_