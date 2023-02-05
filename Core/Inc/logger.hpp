#ifndef LOGGER_HPP_
#define LOGGER_HPP_

class Logger
{
private:
    /* Data at 10mm intervals. */
    uint8_t various_flags_[6200]; //  6.2 [kByte]
    int16_t gyro_data_yaw_[6200]; // 12.4 [kByte]
    float excess_distance_[6200]; // 24.8 [kByte]

public:
    Logger();

};

#endif // LOGGER_HPP_