#include "logger.hpp"
#include <math.h>
#include <string.h>

Logger::Logger() : const_distance_log_(), radian_log_(), various_log_(), const_distance_copy_(), radian_copy_(), various_copy_(), accel_address_(), decel_address_(), periodic_write_enable_(0), accel_position_write_enable_(0), excess_stack_(0), target_velocity_(MIN_VELOCITY) {}

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
    uint8_t accel = 0; // step 0x01, 0x02, 0x03, 0x04, 0x05.
    uint8_t decel = 0; // step 0x10, 0x20, 0x30, 0x40, 0x50.
    uint8_t i;

    for(i = 1; i <= accel_step; i++)
    {
        accel = i;
        address = HEAD_ADDRESS_BLOCK_D + accel_address_[i-1];
        if(!flash.BlankJudgment(address, 1))      return 0x10;
        if(!flash.StoreUint8(address, &accel, 1)) return 0x20;
    }
    for(i = 1; i <= accel_step; i++)
    {
        decel = i << 4;
        address = HEAD_ADDRESS_BLOCK_D + decel_address_[i-1];
        if(!flash.BlankJudgment(address, 1))      return 0x30;
        if(!flash.StoreUint8(address, &decel, 1)) return 0x40;
    }
    
    return 0;
}

void Logger::Loading()
{
    float distance = encoder.GetDistanceStack();

    if(distance < LOGGING_CONST_DISTANCE) return;
    encoder.ResetDistanceStack();

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
    static uint8_t straight_cnt = 0;
    bool straight = false;
    if(fabs(encoder.AngularVelocity()) < STRAIGHT_BORDER_OMEGA){
        if(straight_cnt <= STRAIGHT_JUDGE_COUNT) straight_cnt++;
        else straight = true;
    }else straight_cnt = 0;
    if(corner > pre_corner && straight) run_corner = true;
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

    /* Accel straight */
    uint8_t accel_step = *(reinterpret_cast<uint8_t*>(now_address + HEAD_ADDRESS_BLOCK_D));
    if(!straight) accel_step = 0x10;
    float target = MIN_VELOCITY;
    static float pre_target = MIN_VELOCITY;
    switch(accel_step){
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
        default: target = pre_target; break;
    }
    pre_target = target;
    target_velocity_ = target;

    now_address++;
}

float Logger::GetTargetVelocity()
{
    return target_velocity_;
}