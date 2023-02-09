#include "logger.hpp"
#include <math.h>
#include <string.h>

Logger::Logger() : various_data_(), time_10mm_ms_(), gyro_data_yaw_(), excess_distance_() {}

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
    encoder.ResetDistanceStack();

    static uint16_t now_address = 0; // flash address count. (0 ~ 6199)
    static uint16_t log_index = 0;   // Count up when interrupt.

    /* Mapping data of distance */
    const_distance_log_[log_index] = distance;

    /* Mapping data of radian */
    double degree = iim_42652.GetDegreeStackZ();
    float radian = static_cast<float>(degree * M_PI / 180.0);
    radian_log_[log_index] = radian;
    iim_42652.ResetDegreeStackZ();

    /* Various log */
    static uint8_t pre_corner_cnt = side_sensor.GetCornerMarkerCount();
    static uint8_t pre_cross_cnt  = side_sensor.GetCrossLineCount();
    uint8_t corner_cnt = side_sensor.GetCornerMarkerCount();
    uint8_t cross_cnt  = side_sensor.GetCrossLineCount();
    uint16_t various_buff = 0;
    if(process_complete == 1)        various_buff |= 0x0004;
    if(pre_corner_cnt != corner_cnt) various_buff |= 0x0002;
    if(pre_cross_cnt != cross_cnt)   various_buff |= 0x0001;
    pre_corner_cnt = corner_cnt;
    pre_cross_cnt  = cross_cnt;
    various_log_[log_index] = various_buff;

    /* Copy periodic log */
    if(log_index == LAST_LOG_INDEX)
    {
        log_index = 0;
        memcpy(const_distance_copy_, const_distance_log_, (4 * NUM_OF_LOG));
        memcpy(radian_copy_, radian_log_, (4 * NUM_OF_LOG));
        memcpy(various_copy_, various_log_, (2 * NUM_OF_LOG));
        periodic_write_enable_ = 1;
    }
    else log_index++;

    /* Distance correction */
    uint16_t correction_address = now_address;
    float excess = excess_stack_;
    excess += distance - LOGGING_CONST_DISTANCE;
    if(excess > LOGGING_CONST_DISTANCE)
    {
        const_distance_log_[log_index] = excess;
        radian_log_[log_index] = radian;
        various_log_[log_index] = VARIOUS_LOG_WHEN_COPY;
        excess_stack_ = excess - LOGGING_CONST_DISTANCE;
        now_address++;

        /* Copy periodic log */
        if(log_index == LAST_LOG_INDEX)
        {
            log_index = 0;
            memcpy(const_distance_copy_, const_distance_log_, (4 * NUM_OF_LOG));
            memcpy(radian_copy_, radian_log_, (4 * NUM_OF_LOG));
            memcpy(various_copy_, various_log_, (2 * NUM_OF_LOG));
            periodic_write_enable_ = 1;
        }
        else log_index++;
    }
    else excess_stack_ = excess;

    /* Accel position log */
    static uint16_t accel_straight_cnt = 0;
    static uint8_t accel_step = 0;
    if(fabs(encoder.AngularVelocity()) < STRAIGHT_BORDER_OMEGA)
    {
        accel_straight_cnt++;
        if(accel_straight_cnt >= CNT_OF_ACCEL_STEP_UP && accel_step < NUM_OF_ACCEL_STEP)
        {
            accel_step++;
            accel_address_[accel_step-1] = correction_address - CNT_OF_ACCEL_STEP_UP;
            accel_straight_cnt = 0;
        }
    }
    if else(accel_step != 0)
    {
        for(int i = 1; i <= accel_step; i++)
        {
            decel_address_[i-1] = (correction_address-1) - (DIFF_NEXT_ACCEL_STEP * i);
        }
        accel_position_write_enable_ = accel_step;
        accel_step = 0;
        accel_straight_cnt = 0;
    }
    else accel_straight_cnt = 0;

    now_address++;
}

uint8_t Logger::StorePeriodicLog()
{
    if(periodic_write_enable_ == 0) return 0;
    else if(periodic_write_enable_ == 1) periodic_write_enable_ = 0;
    else return 0xFF;

    static uint32_t address_a = HEAD_ADDRESS_BLOCK_A;
    static uint32_t address_b = HEAD_ADDRESS_BLOCK_B;
    static uint32_t address_c = HEAD_ADDRESS_BLOCK_C;
    uint8_t result = 0;

    if(!flash.BlankJudgeWord(address_a, NUM_OF_LOG))     return 0x10;
    if(!flash.BlankJudgeWord(address_b, NUM_OF_LOG))     return 0x20;
    if(!flash.BlankJudgeHalfword(address_c, NUM_OF_LOG)) return 0x40;

    if(!flash.StoreFloat(address_a, const_distance_copy_, NUM_OF_LOG)) result |= 0x01;
    if(!flash.StoreFloat(address_b, radian_copy_, NUM_OF_LOG))         result |= 0x02;
    if(!flash.StoreUint16(address_c, various_copy_, NUM_OF_LOG))       result |= 0x04;

    address_a += NUM_OF_LOG * 4;
    address_b += NUM_OF_LOG * 4;
    address_c += NUM_OF_LOG * 2;

    return result;
}

uint8_t Logger::StoreAccelPositionLog()
{
    uint8_t accel_step = accel_position_write_enable_;
    if(accel_step == 0) return 0;
    else accel_position_write_enable_ = 0;

    uint32_t address;
    uint8_t accel = ACCEL_CODE;
    uint8_t decel = DECEL_CODE;
    uint8_t i;

    for(i = 1; i <= accel_step; i++)
    {
        address = HEAD_ADDRESS_BLOCK_D + accel_address_[i-1];
        if(!flash.BlankJudgment(address, 1))      return 0x10;
        if(!flash.StoreUint8(address, &accel, 1)) return 0x20;
    }
    for(i = 1; i <= accel_step; i++)
    {
        address = HEAD_ADDRESS_BLOCK_D + decel_address_[i-1];
        if(!flash.BlankJudgment(address, 1))      return 0x30;
        if(!flash.StoreUint8(address, &decel, 1)) return 0x40;
    }
    
    return 0;
}

void Logger::Loading()
{
    static uint16_t now_address = 0;

    /* Load curvature radius */
    float log_distance = *(reinterpret_cast<float*>(now_address + HEAD_ADDRESS_BLOCK_A));
    float log_radian = *(reinterpret_cast<float*>(now_address + HEAD_ADDRESS_BLOCK_B));
    float radius = log_distance / log_radian;

    /* Load correction position */
    uint16_t log_various = *(reinterpret_cast<uint16_t*>(now_address + HEAD_ADDRESS_BLOCK_C));
    bool log_corner = false, log_corner_pass = false;
    bool log_cross = false, log_cross_pass = false;
    if(log_various & 0x0002 == 0x0002) log_corner = true;
    if(log_various & 0x0001 == 0x0001) log_cross = true;

    /* Get correction position */
    bool run_corner = false, run_corner_pass = false;
    bool run_cross = false, run_cross_pass = false;
    uint8_t corner = side_sensor.GetCornerMarkerCount();
    uint8_t cross = side_sensor.GetCrossLineCount();
    static uint8_t pre_corner = corner;
    static uint8_t pre_cross = cross;
    if(corner > pre_corner) run_corner = true;
    if(cross > pre_cross) run_cross = true;
    pre_corner = corner;
    pre_cross = cross;

    /* Position correction at corner marker */
    static uint16_t log_corner_address, run_corner_address;
    static uint8_t log_corner_wait = 0, run_corner_wait = 0;
    if(log_corner == true){
        if(run_corner_pass == true){
            now_address += now_address - run_corner_address;
            run_corner_wait = 0;
            run_corner_pass = false;
        }else{
            log_corner_address = now_address;
            log_corner_pass = true;
            log_corner_wait = CORRECTION_WAIT_COUNT;
        }
    }else{
        if(run_corner_wait > 0) run_corner_wait--;
        else run_corner_pass = false;
    }
    if(run_corner == true){
        if(log_corner_pass == true){
            now_address = log_corner_address;
            log_corner_wait = 0;
            log_corner_pass = false;
        }else{
            run_corner_address = now_address;
            run_corner_pass = true;
            run_corner_wait = CORRECTION_WAIT_COUNT;
        }
    }else{
        if(log_corner_wait > 0) log_corner_wait--;
        else log_corner_pass = false;
    }

    /* Position correction at cross line */
    static uint16_t log_cross_address, run_cross_address;
    static uint8_t log_cross_wait = 0, run_cross_wait = 0;
    if(log_cross == true){
        if(run_cross_pass == true){
            now_address += now_address - run_cross_address;
            run_cross_wait = 0;
            run_cross_pass = false;
        }else{
            log_cross_address = now_address;
            log_cross_pass = true;
            log_cross_wait = CORRECTION_WAIT_COUNT;
        }
    }else{
        if(run_cross_wait > 0) run_cross_wait--;
        else run_cross_pass = false;
    }
    if(run_cross == true){
        if(log_cross_pass == true){
            now_address = log_cross_address;
            log_cross_wait = 0;
            log_cross_pass = false;
        }else{
            run_cross_address = now_address;
            run_cross_pass = true;
            run_cross_wait = CORRECTION_WAIT_COUNT;
        }
    }else{
        if(log_cross_wait > 0) log_cross_wait--;
        else log_cross_pass = false;
    }







}