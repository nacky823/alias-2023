#include "logger.hpp"

Logger::Logger()

void Log::Logging(uint8_t interrupt_end)
{
    float distance_10mm = encoder.GetDistance10mm();

    if(distance_10mm > FORMAL_10MM)
    {
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
    
        if(log_index < MAX_LOG_INDEX) log_index++;
    }
}
