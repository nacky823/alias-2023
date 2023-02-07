#include "logger.hpp"
#include <math.h>

Logger::Logger() : various_data_(), time_10mm_ms_(), gyro_data_yaw_(), excess_distance_() {}

void Logger::Logging(uint8_t interrupt_end)
{
    float distance_10mm = encoder.GetDistance10mm();

    if(distance_10mm > FORMAL_10MM)
    {
        time_10mm_ms_[log_index] = static_cast<uint16_t>(g_count_100us);
        g_count_100us = 0;

        static uint16_t log_index = 0;
        static uint8_t pre_goal_cnt   = side_sensor.GetGoalMarkerCount();
        static uint8_t pre_corner_cnt = side_sensor.GetCornerMarkerCount();
        static uint8_t pre_cross_cnt  = side_sensor.GetCrossLineCount();
        uint8_t goal_cnt   = side_sensor.GetGoalMarkerCount();
        uint8_t corner_cnt = side_sensor.GetCornerMarkerCount();
        uint8_t cross_cnt  = side_sensor.GetCrossLineCount();
        uint16_t various_buff = 0;

        if(interrupt_end == 1)           various_buff |= 0x0008;
        if(pre_goal_cnt != goal_cnt)     various_buff |= 0x0004;
        if(pre_corner_cnt != corner_cnt) various_buff |= 0x0002;
        if(pre_cross_cnt != cross_cnt)   various_buff |= 0x0001;
    
        pre_goal_cnt   = goal_cnt;
        pre_corner_cnt = corner_cnt;
        pre_cross_cnt  = cross_cnt;

        various_data_[log_index]    = various_buff;
        gyro_data_yaw_[log_index]   = iim_42652.GyroZLeft();
        excess_distance_[log_index] = distance_10mm - FORMAL_10MM;

        if(log_index >= (LOG_MAX_CNT_10MM - 1))
        {
            memcpy(various_copy_, various_data_, (2 * LOG_MAX_CNT_10MM));
            memcpy(time_10mm_copy_, time_10mm_ms_, (2 * LOG_MAX_CNT_10MM));
            memcpy(gyro_yaw_copy_, gyro_data_yaw_, (2 * LOG_MAX_CNT_10MM));
            memcpy(excess_copy_, excess_distance_, (4 * LOG_MAX_CNT_10MM));
            store_log_flag_ = 1;
            log_index = 0;
        }
    
        encoder.ResetDistance10mm();
        log_index++;
    }
}

void Logger::StoreLog()
{
    if(store_log_flag_ == 0) return;
    else if(store_log_flag_ == 1) store_log_flag_ = 0;

    static uint32_t address_s1 = SECTOR_1_ADDRESS_HEAD;
    static uint32_t address_s2 = SECTOR_2_ADDRESS_HEAD;
    static uint32_t address_s3 = SECTOR_3_ADDRESS_HEAD;
    static uint32_t address_s4 = SECTOR_4_ADDRESS_HEAD;
    uint8_t error = 0;

    if(!flash.StoreUint16(address_s1, various_copy_, LOG_MAX_CNT_10MM)) error = 1;
    if(!flash.StoreUint16(address_s2, time_10mm_copy_, LOG_MAX_CNT_10MM)) error = 1;
    if(!flash.StoreInt16(address_s3, gyro_yaw_copy_, LOG_MAX_CNT_10MM)) error = 1;
    if(!flash.StoreFloat(address_s4, excess_copy_, LOG_MAX_CNT_10MM)) error = 1;

    address_s1 += (2 * LOG_MAX_CNT_10MM);
    address_s2 += (2 * LOG_MAX_CNT_10MM);
    address_s3 += (2 * LOG_MAX_CNT_10MM);
    address_s4 += (4 * LOG_MAX_CNT_10MM);

    if(error == 1) led.ColorOrder('R');
}

float Logger::TargetVelocity(float distance)
{
    double degree = imu.GetDegreeStackZ(); // [deg]
    double radian = degree * M_PI / 180.0; // [rad]
    double radius = distance / radian;     // [mm]

    if(radius < 100) return TARGET_V_R10;
    else if(radius < 300) return TARGET_V_R30;
    else if(radius < 500) return TARGET_V_R50;
    else if(radius < 800) return TARGET_V_R80;
    else if(radius < 1000) return TARGET_V_R100;
}

{
    float distance = encoder.GetDistance10mm();

    if(distance < FORMAL_10MM) return;

    static uint16_t call_cnt = 0;


}


{
    float distance_10mm = encoder.GetDistance10mm();

    if(distance_10mm < FORMAL_10MM) return;
    static uint16_t inter_cnt = 0; // index 0

    // log data
    uint16_t various, time_10mm;
    int16_t gyro_yaw;
    float excess;
    uint32_t ref_adr_s1 = SECTOR_1_ADDRESS_HEAD + (intr_cnt * 2); // 参照場所
    uint32_t ref_adr_s2 = SECTOR_2_ADDRESS_HEAD + (intr_cnt * 2);
    uint32_t ref_adr_s3 = SECTOR_3_ADDRESS_HEAD + (intr_cnt * 2);
    uint32_t ref_adr_s4 = SECTOR_4_ADDRESS_HEAD + (intr_cnt * 4);
    memcpy(&various, ref_adr_s1, 2);
    memcpy(&time_10mm, ref_adr_s2, 2);
    memcpy(&gyro_yaw, ref_adr_s3, 4);
    memcpy(&excess, ref_adr_s4, 4);
    
    double omega = gyro_yaw * NORMAL_FUCOR_DPS;
    double degree = fabs(omega * TIM7_PERIOD * time_10mm);

    if(degree > )


    float now_position;

    uint16_t now_index

    now_index_ = index_num;

    log_index_

    refer_adderss_s1 



    float log_10mm  = FORMAL_10MM + excess;
    float now_10mm  = distance_10mm;
    float init_10mm = log_10mm - now_10mm;

    encoder.SetDistance10mm(init_10mm);



        
        
    
}

float Logger::FindTargetVelocity()
{
    uint16_t various, time_10mm;
    int16_t gyro_yaw;
    float excess;

    static uint16_t log_index = 0;
    uint32_t address_s1 = SECTOR_1_ADDRESS_HEAD + (log_index * 2);
    uint32_t address_s2 = SECTOR_2_ADDRESS_HEAD + (log_index * 2);
    uint32_t address_s3 = SECTOR_3_ADDRESS_HEAD + (log_index * 2);
    uint32_t address_s4 = SECTOR_4_ADDRESS_HEAD + (log_index * 4);

    memcpy(&various, address_s1, 2);
    memcpy(&time_10mm, address_s2, 2);
    memcpy(&gyro_yaw, address_s3, 2);
    memcpy(&excess, address_s4, 4);



}

void Logger::Logging(uint8_t process_complete)
{
    float distance = encoder.GetDistanceStack();

    if(distance < LOGGING_CONST_DISTANCE) return;

    static uint16_t log_index = 0; // Count up when interrupt.

    /* Mapping data of distance. */
    const_distance_log_[log_index] = distance;

    /* Mapping data of radian. */
    double degree = iim_42652.GetDegreeStackZ();
    double radian = degree * M_PI / 180.0;
    radian_log_[log_index] = static_cast<double>(radian);

    /* Various log */
    static uint8_t pre_goal_cnt   = side_sensor.GetGoalMarkerCount();
    static uint8_t pre_corner_cnt = side_sensor.GetCornerMarkerCount();
    static uint8_t pre_cross_cnt  = side_sensor.GetCrossLineCount();
    uint8_t goal_cnt   = side_sensor.GetGoalMarkerCount();
    uint8_t corner_cnt = side_sensor.GetCornerMarkerCount();
    uint8_t cross_cnt  = side_sensor.GetCrossLineCount();
    uint16_t various_buff = 0;
    if(process_complete == 1)        various_buff |= 0x0008;
    if(pre_goal_cnt != goal_cnt)     various_buff |= 0x0004;
    if(pre_corner_cnt != corner_cnt) various_buff |= 0x0002;
    if(pre_cross_cnt != cross_cnt)   various_buff |= 0x0001;
    pre_goal_cnt   = goal_cnt;
    pre_corner_cnt = corner_cnt;
    pre_cross_cnt  = cross_cnt;
    various_log_[log_index] = various_buff;

    if(log_index == LAST_LOG_INDEX)
    {
        memcpy(various_copy_, various_log_, (2 * NUM_OF_LOG));
        memcpy(radian_copy_, radian_log_, (4 * NUM_OF_LOG));
        memcpy(const_distance_copy_, const_distance_log_, (4 * NUM_OF_LOG));
        log_index = 0;
        flash_write_enable_ = 1;
    }
    else log_index++;

    encoder.ResetDistanceStack();
    iim_42652.ResetDegreeStackZ();
}

void Logger::StoreLog()
{
    if(flash_write_enable_ == 0) return;
    else if(flash_write_enable_ == 1) flash_write_enable_ = 0;

    static uint32_t address_a = HEAD_ADDRESS_BLOCK_A;
    static uint32_t address_b = HEAD_ADDRESS_BLOCK_B;
    static uint32_t address_c = HEAD_ADDRESS_BLOCK_C;
    uint8_t error = 0;

    if(!flash.StoreUint16(address_a, various_copy_, NUM_OF_LOG)) error = 1;
    if(!flash.StoreFloat(address_b, radian_copy_, NUM_OF_LOG)) error = 1;
    if(!flash.StoreFloat(address_c, const_distance_copy_, NUM_OF_LOG)) error = 1;

    address_a += (2 * NUM_OF_LOG);
    address_b += (4 * NUM_OF_LOG);
    address_c += (4 * NUM_OF_LOG);

    if(error == 1) led.ColorOrder('R');
}

float Logger::TargetVelocity(float distance)
{
    double degree = imu.GetDegreeStackZ(); // [deg]
    double radian = degree * M_PI / 180.0; // [rad]



    double radius = distance / radian;     // [mm]

    if(radius < 100) return TARGET_V_R10;
    else if(radius < 300) return TARGET_V_R30;
    else if(radius < 500) return TARGET_V_R50;
    else if(radius < 800) return TARGET_V_R80;
    else if(radius < 1000) return TARGET_V_R100;
}





