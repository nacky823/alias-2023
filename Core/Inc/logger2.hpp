#ifndef LOGGER2_HPP_
#define LOGGER2_HPP_

#include "macro.h"
#include "encoder.hpp"
#include "flash.hpp"
#include "iim_42652.hpp"
#include "side_sensor.hpp"

/* Address to store error code when flash write fails. */
#define ERROR_CODE_STORE_DISTANCE 0x00
#define ERROR_CODE_STORE_RADIAN   0x00
#define ERROR_CODE_STORE_VARIOUS  0x00
#define ERROR_CODE_STORE_ACCEL    0x00
#define ERROR_CODE_STORE_DECEL    0x00

#define LOGGING_CONST_DISTANCE 20.0 // [mm]
#define NUM_OF_LOG     10
#define LAST_LOG_INDEX 9
#define STRAIGHT_BORDER_OMEGA 404 //temp
#define STRAIGHT_BORDER_ENCODER 1.0
#define NUM_OF_ACCEL_STEP 5
#define DIFF_NEXT_ACCEL_STEP 4
#define CNT_OF_ACCEL_STEP_UP 8
#define ACCEL_VELOCITY 0.5
#define CORRECTION_WAIT_COUNT 3
#define STRAIGHT_JUDGE_COUNT 5
#define VARIOUS_LOG_DATA_WHEN_COPY 0xF0F0

#define HEAD_ADDRESS_BLOCK_A SECTOR_2_ADDRESS_HEAD
#define HEAD_ADDRESS_BLOCK_B SECTOR_3_ADDRESS_HEAD
#define HEAD_ADDRESS_BLOCK_C SECTOR_4_ADDRESS_HEAD
#define HEAD_ADDRESS_BLOCK_D SECTOR_1_ADDRESS_HEAD

class Logger2
{
private:
    bool emergency_stop_flag_;
    bool success_emergency_code_store_;
    float logging_radian_buff_;
    uint16_t loading_now_address_;
    uint16_t uncorrected_address_buff_;
    uint16_t logging_now_address_;
    float target_velocity_;

    Encoder *encoder_;
    Flash *flash_;
    Iim42652 *iim_42652_;
    SideSensor *side_sensor_;

public:
    Logger2(Encoder *,
            Flash *,
            Iim42652 *,
            SideSensor *);
    void Logging();
    bool AssertNowaddress(uint16_t);
    uint8_t StoreDistanceLog(float);
    uint8_t StoreRadianLog();
    uint8_t StoreVariousLog();
    void DistanceCorrection(float);
    void CheckLoggingSuccess(uint32_t, uint8_t);
    void SetEmergencyStopFlag(bool);
    void SetSuccessEmergencyCodeStore(bool);
    bool GetEmergencyStopFlag();
    bool GetSuccessEmergencyCodeStore();
    void LoggingAccelPosition();
    uint8_t StoreAccelPosition(uint32_t, uint8_t);
    void Loading();
    void AccelStraight();
    float GetTargetVelocity();
    void ResetLoadingNowAddress();
    void ResetLoggingNowAddress();

};

#endif // LOGGER2_HPP_