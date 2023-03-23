#ifndef LOGGER_HPP_
#define LOGGER_HPP_

#include "macro.h"
#include "encoder.hpp"
#include "flash.hpp"
#include "iim_42652.hpp"
#include "side_sensor.hpp"

#define LOGGING_CONST_DISTANCE 20.0 // [mm]
#define NUM_OF_LOG     10
#define LAST_LOG_INDEX 9
#define STRAIGHT_BORDER_OMEGA 404 //temp
#define NUM_OF_ACCEL_STEP 5
#define DIFF_NEXT_ACCEL_STEP 2
#define CNT_OF_ACCEL_STEP_UP 4
#define ACCEL_VELOCITY 0.5
#define CORRECTION_WAIT_COUNT 3
#define STRAIGHT_JUDGE_COUNT 5
#define VARIOUS_LOG_DATA_WHEN_COPY 0xF0F0

#define HEAD_ADDRESS_BLOCK_A SECTOR_2_ADDRESS_HEAD
#define HEAD_ADDRESS_BLOCK_B SECTOR_3_ADDRESS_HEAD
#define HEAD_ADDRESS_BLOCK_C SECTOR_4_ADDRESS_HEAD
#define HEAD_ADDRESS_BLOCK_D SECTOR_1_ADDRESS_HEAD

class Logger
{
private:
    /* Periodic log at 20mm intervals */
    float const_distance_log_[NUM_OF_LOG]; // total 12.4k[Byte]
    float radian_log_[NUM_OF_LOG];         // total 12.4k[Byte]
    uint16_t various_log_[NUM_OF_LOG];     // total  6.2k[Byte]
    /* Buffer */
    float const_distance_copy_[NUM_OF_LOG];
    float radian_copy_[NUM_OF_LOG];
    uint16_t various_copy_[NUM_OF_LOG];

    /* Accel position log */
    uint16_t accel_address_[NUM_OF_ACCEL_STEP];
    uint16_t decel_address_[NUM_OF_ACCEL_STEP];

    uint8_t periodic_write_enable_;
    uint8_t accel_position_write_enable_;
    float excess_stack_;
    float target_velocity_;

    Encoder encoder;
    Flash flash;
    Iim42652 iim_42652;
    SideSensor side_sensor;

public:
    Logger();
    void Logging(uint8_t);
    uint8_t StorePeriodicLog();
    uint8_t StoreAccelPositionLog();
    void Loading();
    float GetTargetVelocity();

};

#endif // LOGGER_HPP_