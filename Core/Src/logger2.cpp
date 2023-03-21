#include "logger2.hpp"

void Logger2::Logging()
{
    float distance = encoder_->GetDistanceStack();

    if(distance < LOGGING_CONST_DISTANCE) return;

    encoder_->ResetDistanceStack();

    static uint16_t now_address = 0; // flash address count (0 ~ 6199)



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
    else if(!flash_->StoreFloat(address, distance, 1)) result = 0x03;

    now_address++;

    return result;
}

uint8_t Logger2::StoreRadianLog()
{
    static uint32_t address = HEAD_ADDRESS_BLOCK_B;
    uint8_t result = 0;

    double degree = iim_42652_->GetDegreeStackZ();
    float radian = static_cast<float>(degree * M_PI / 180.0);
    iim_42652_->ResetDegreeStackZ();

    if(!flash_->CheckBlankWord(address, 1)) result = 0x01;
    else if(!flash_->StoreFloat(address, radian, 1)) result = 0x02;

    address += 4;

    return result;
}

uint8_t Logger2::StoreVariousLog()
{
    static uint32_t address = HEAD_ADDRESS_BLOCK_C;
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

    if(!flash_->CheckBlankHalfword(address, 1)) result = 0x01;
    else if(!flash_->StoreUint16(address, various, 1)) result = 0x02;

    address += 4;

    return result;
}

uint16_t Logger2::DistanceCorrection(float distance)
{
    uncorrected_address_buff_ = logging_now_address_;

    static float excess_stack = 0;
    excess_stack += distance - LOGGING_CONST_DISTANCE;

    if(excess_stack > LOGGING_CONST_DISTANCE)
    {
        uint8_t result_store_distance = StoreDistanceLog(distance);

        if(result_store_distance != 0)
        {
            SetEmergencyStopFlag();
        }



    }
}

void Logger2::SetEmergencyStopFlag()
{
    emergency_stop_flag_ = true;
}













