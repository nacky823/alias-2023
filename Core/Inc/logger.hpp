#ifndef LOGGER_HPP_
#define LOGGER_HPP_

#include "encoder.hpp"
#include "side_sensor.hpp"
#include "iim_42652.hpp"

#define LOG_MAX_CNT_10MM 6200
#define FORMAL_10MM      10.0
#define NORMAL_FUCOR_DPS 0.0610370 // 2000 / 32768
#define NUM_OF_ACCEL_STEP 5
#define CNT_OF_ACCEL_STEP_UP 16
#define ACCEL_NEXT_STEP_DIFF 8

class Logger
{
private:
    /* Log at 20mm intervals */
    float const_distance_log_[NUM_OF_LOG]; // total 12.4k[Byte]
    float radian_log_[NUM_OF_LOG];         // total 12.4k[Byte]
    uint16_t various_log_[NUM_OF_LOG];     // total  6.2k[Byte]

    /* Accel position log */
    uint16_t accel_address_[NUM_OF_ACCEL_STEP];
    uint16_t decel_address_[NUM_OF_ACCEL_STEP];

    float excess_stack_;

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