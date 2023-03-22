#include "logger2.hpp"
#include <math.h>

Logger2::Logger2()

void Logger2::Logging()
{
    float distance = encoder_->GetDistanceStack();
    if(distance < LOGGING_CONST_DISTANCE) return;
    encoder_->ResetDistanceStack();

    uint8_t result = StoreDistanceLog(distance);
    CheckLoggingSuccess(ERROR_CODE_STORE_DISTANCE, result);

    result = StoreRadianLog();
    CheckLoggingSuccess(ERROR_CODE_STORE_RADIAN, result);

    result = StoreVariousLog();
    CheckLoggingSuccess(ERROR_CODE_STORE_VARIOUS, result);

    DistanceCorrection(distance);

    LoggingAccelPosition();

    logging_now_address_++;
}

bool Logger2::AssertNowaddress(uint16_t now_address)
{
    if(now_address == logging_now_address_) return true;
    else return false;
}

uint8_t Logger2::StoreDistanceLog(float distance)
{
    static uint16_t now_address = 0;
    uint32_t address = now_address * 4 + HEAD_ADDRESS_BLOCK_A;
    uint8_t result = 0;

    if(!AssertNowaddress(now_address)) result = 0x01;
    else if(!flash_->CheckBlankWord(address, 1)) result = 0x02;
    else if(!flash_->StoreFloat(address, &distance, 1)) result = 0x03;

    now_address++;

    return result;
}

uint8_t Logger2::StoreRadianLog()
{
    static uint16_t now_address = 0;
    uint32_t address = now_address * 4 + HEAD_ADDRESS_BLOCK_B;
    uint8_t result = 0;

    double degree = iim_42652_->GetDegreeStackZ();
    float radian = static_cast<float>(degree * M_PI / 180.0);
    iim_42652_->ResetDegreeStackZ();
    logging_radian_buff_ = radian;

    if(!AssertNowaddress(now_address)) result = 0x01;
    else if(!flash_->CheckBlankWord(address, 1)) result = 0x02;
    else if(!flash_->StoreFloat(address, &radian, 1)) result = 0x03;

    now_address++;

    return result;
}

uint8_t Logger2::StoreVariousLog()
{
    static uint16_t now_address = 0;
    uint32_t address = now_address * 2 + HEAD_ADDRESS_BLOCK_C;
    uint8_t result = 0;

    uint8_t corner_count = side_sensor_->GetCornerMarkerCount();
    uint8_t cross_count  = side_sensor_->GetCrossLineCount();
    static uint8_t pre_corner_count = corner_count;
    static uint8_t pre_cross_count  = cross_count;
    uint16_t various = 0;

    if(pre_corner_count != corner_count) various |= 0x0002;
    if(pre_cross_count != cross_count)   various |= 0x0001;

    pre_corner_count = corner_count;
    pre_cross_count  = cross_count;

    if(!AssertNowaddress(now_address)) result = 0x01;
    else if(!flash_->CheckBlankHalfword(address, 1)) result = 0x02;
    else if(!flash_->StoreUint16(address, &various, 1)) result = 0x03;

    now_address++;

    return result;
}

void Logger2::DistanceCorrection(float distance)
{
    uncorrected_address_buff_ = logging_now_address_;

    static float excess_stack = 0;
    excess_stack += distance - LOGGING_CONST_DISTANCE;

    if(excess_stack > LOGGING_CONST_DISTANCE)
    {
        uint8_t result = StoreDistanceLog(distance);
        CheckLoggingSuccess(ERROR_CODE_STORE_DISTANCE, result);

        excess_stack -= LOGGING_CONST_DISTANCE;

        logging_now_address_++;
    }
}

void Logger2::CheckLoggingSuccess(uint32_t address, uint8_t data)
{
    if(data != 0)
    {
        SetEmergencyStopFlag(true);

        if(flash_->StoreUint8(address, &data, 1))
        {
            SetSuccessEmergencyCodeStore(true);
        }
    }
}

void Logger2::SetEmergencyStopFlag(bool boolean)
{
    emergency_stop_flag_ = boolean;
}

bool Logger2::GetEmergencyStopFlag()
{
    return emergency_stop_flag_;
}

void Logger2::SetSuccessEmergencyCodeStore(bool boolean)
{
    success_emergency_code_store_ = boolean;
}

bool Logger2::GetSuccessEmergencyCodeStore()
{
    return success_emergency_code_store_;
}

void Logger2::LoggingAccelPosition()
{
    static uint16_t accel_straight_count = 0;
    static uint8_t accel_step = 0;

    if(fabs(encoder_->AngularVelocity()) < STRAIGHT_BORDER_ENCODER)
    {
        accel_straight_count++;
        if(accel_straight_count >= CNT_OF_ACCEL_STEP_UP && accel_step < NUM_OF_ACCEL_STEP)
        {
            accel_step++;

            uint32_t accel_address = uncorrected_address_buff_ - CNT_OF_ACCEL_STEP_UP;

            uint8_t result = StoreAccelPosition(accel_address, accel_step);

            CheckLoggingSuccess(ERROR_CODE_STORE_ACCEL, result);

            accel_straight_count = 0;
        }
    }
    else if(accel_step != 0)
    {
        for(uint8_t i = 1; i <= accel_step; i++)
        {
            uint32_t decel_address = (uncorrected_address_buff_-1) - (DIFF_NEXT_ACCEL_STEP * i);
            uint8_t decel_step = i << 4;

            uint8_t result = StoreAccelPosition(decel_address, decel_step);

            CheckLoggingSuccess(ERROR_CODE_STORE_DECEL, result);
        }
        accel_step = 0;
        accel_straight_count = 0;
    }
    else accel_straight_count = 0;
}

uint8_t Logger2::StoreAccelPosition(uint32_t address, uint8_t data)
{
    address += HEAD_ADDRESS_BLOCK_D;

    if(!flash_->CheckBlankByte(address, 1)) return 0x01;
    else if(!flash_->StoreUint8(address, &data, 1)) return 0x02;

    return 0;
}

void Logger2::Loading()
{
    float distance = encoder_->GetDistanceStack();
    if(distance < LOGGING_CONST_DISTANCE) return;
    encoder_->ResetDistanceStack();

    void AccelStraight();

    loading_now_address_++;
}

void Logger2::AccelStraight()
{
    static uint16_t now_address = 0;
    now_address = loading_now_address_ + HEAD_ADDRESS_BLOCK_D;

    uint8_t accel_step = *(reinterpret_cast<uint8_t*>(now_address));

    static float pre_target = MIN_VELOCITY;
    float target;

    switch(accel_step)
    {
        case 0x10: target = MIN_VELOCITY; break;
        case 0x01:
        case 0x20: target = MIN_VELOCITY + (ACCEL_VELOCITY * 1); break;
        case 0x02:
        case 0x30: target = MIN_VELOCITY + (ACCEL_VELOCITY * 2); break;
        case 0x03:
        case 0x40: target = MIN_VELOCITY + (ACCEL_VELOCITY * 3); break;
        case 0x04:
        case 0x50: target = MIN_VELOCITY + (ACCEL_VELOCITY * 4); break;
        case 0x05: target = MIN_VELOCITY + (ACCEL_VELOCITY * 5); break;
        default:   target = pre_target; break;
    }

    if(fabs(encoder_->AngularVelocity()) > STRAIGHT_BORDER_ENCODER)
    {
        target = MIN_VELOCITY;
    }

    target_velocity_ = target;
    pre_target = target;
}

float Logger2::GetTargetVelocity()
{
    return target_velocity_;
}

void Logger2::ResetLoadingNowAddress()
{
    loading_now_address_ = 0;
}

void Logger2::ResetLoggingNowAddress()
{
    logging_now_address_ = 0;
}