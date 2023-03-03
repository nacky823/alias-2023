#include "side_sensor.hpp"
#include "declare_extern.h"

SideSensor::SideSensor() : read_state_flags_(0)
                         , write_state_flags_(0)
                         , exception_flags_(0)
                         , master_count_(0)
                         , goal_marker_count_(0)
                         , corner_marker_count_(0)
                         , cross_line_count_(0) {}

void SideSensor::UpdateState()
{
    uint8_t io_state = 0xFF; // Lower bits represent the positions of sensors.

    if(IO_OUTSIDE_L == 0) io_state &= 0xF7; // 0111
    if(IO_INSIDE_L == 0)  io_state &= 0xFB; // 1011
    if(IO_INSIDE_R == 0)  io_state &= 0xFD; // 1101
    if(IO_OUTSIDE_R == 0) io_state &= 0xFE; // 1110

    read_state_flags_ <<= 4;

    switch(io_state)
    {
        case 0xFF: // 1111
            read_state_flags_ |= 0x08; break;
        case 0xFC: // 1100  If through goal, increase pattern.
            read_state_flags_ |= 0x04; break;
        case 0xFB: // 1011
        case 0xF7: // 0111
        case 0xF3: // 0011
            read_state_flags_ |= 0x02; break;
        case 0xF6: // 0110
        case 0xF2: // 0010
        case 0xF4: // 0100
        case 0xF0: // 0000
        case 0xF1: // 0001
        case 0xF8: // 1000
        case 0xF9: // 1001
            read_state_flags_ |= 0x01; break;
        default: break;
    }
}

void SideSensor::IgnoreJudgment()
{
    uint8_t ignore_flag = exception_flags_ & 0x01;

    if(ignore_flag == 0x01)
    {
        if(master_count_ < IGNORE_COUNT) master_count_++;
        else exception_flags_ >>= 1; 
    }
    else if(ignore_flag == 0x00)
    {
        UpdateState();

        uint8_t now_state = read_state_flags_;
        uint8_t pre_state = (now_state << 4) | (now_state >> 4);

        if(pre_state != now_state)
        {
            NoiseTolerance();
        }
        else if(pre_state == now_state)
        {
            exception_flags_ &= 0xF3; // noise_count = 0
            ConfirmState();
            CountUp();
        }
    }

#ifdef DEBUG_MODE
    MonitorFlags();
#endif // DEBUG_MODE
}

void SideSensor::NoiseTolerance()
{
    uint8_t noise_count = (exception_flags_ & 0x0C) >> 2;

    if(noise_count == 0x00)
    {
        exception_flags_ = (exception_flags_ & 0x0F) | (read_state_flags_ & 0xF0);
        noise_count++;
        exception_flags_ = (exception_flags_ & 0xF3) | (noise_count << 2);
    }
    else if(noise_count == 0x01)
    {
        noise_count++;
        exception_flags_ = (exception_flags_ & 0xF3) | (noise_count << 2);
        ConfirmState();
        CountUp();
    }
    else master_count_ = 0;
}

void SideSensor::ConfirmState()
{
    uint8_t before_noise_state = exception_flags_ >> 4;
    uint8_t now_state = read_state_flags_ & 0x0F;

    if(before_noise_state != now_state)
    {
        exception_flags_ = (exception_flags_ & 0x0F) | (now_state << 4);
        master_count_ = 0;
    }
    else if(master_count_ < MAX_INTERRUPT_COUNT)
    {
        master_count_++;
    }

    if(now_state == 0x08 && master_count_ >= BLACK_BLACK_COUNT)
    {
        write_state_flags_ = (write_state_flags_ & 0xF0) | 0x08;
    }
    else if(now_state == 0x04 && master_count_ >= BLACK_WHITE_COUNT)
    {
        write_state_flags_ |= 0x04;
    }
    else if(now_state == 0x02 && master_count_ >= WHITE_BLACK_COUNT)
    {
        write_state_flags_ |= 0x02;
    }
    else if(now_state == 0x01 && master_count_ >= WHITE_WHITE_COUNT)
    {
        write_state_flags_ = (write_state_flags_ & 0x28) | 0x01;
        exception_flags_ = (exception_flags_ << 8) | 0x01;
        master_count_ = 0;
    }
}

void SideSensor::CountUp()
{
    uint8_t state = write_state_flags_;
    uint8_t black_flag   = (state & 0x08) >> 3;
    uint8_t goal_flag    = (state & 0x04) >> 2;
    uint8_t corner_flag  = (state & 0x02) >> 1;
    uint8_t cross_flag   =  state & 0x01;
    uint8_t goal_reach   = (state & 0x80) >> 7;
    uint8_t corner_reach = (state & 0x40) >> 6;
    uint8_t cross_reach  = (state & 0x20) >> 5;
    
    if(goal_flag == 0x01 && black_flag == 0x01)
    {
        state &= 0xF7; // black_flag = false
        state |= 0x80; // goal_reach = true
    }
    else if(goal_reach == 0x01 && black_flag == 0x01)
    {
        state &= 0x7F; // goal_reach = false
        goal_marker_count_++;
    }
    else if(corner_flag == 0x01 && black_flag == 0x01)
    {
        state &= 0xF7; // black_flag = false
        state |= 0x40; // corner_reach = true
    }
    else if(corner_reach == 0x01 && black_flag == 0x01)
    {
        state &= 0xBF; // corner_reach = false
        corner_marker_count_++;
    }
    else if(cross_flag == 0x01 && black_flag == 0x01)
    {
        state &= 0xF7; // black_flag = false
        state |= 0x20; // cross_reach = true
    }
    else if(cross_reach == 0x01 && black_flag == 0x01)
    {
        state &= 0xDF // cross_reach = false
        cross_line_count_++;
    }

    write_state_flags_ = state;
}

uint8_t SideSensor::GetGoalMarkerCount()
{
    return goal_marker_count_;
}

uint8_t SideSensor::GetCornerMarkerCount()
{
    return corner_marker_count_;
}

uint8_t SideSensor::GetCrossLineCount()
{
    return cross_line_count_;
}

#ifdef DEBUG_MODE
void SideSensor::MonitorFlags()
{
    /* read_state_flags_ */
    g_side_pre_state = (read_state_flags_ & 0xF0) >> 4;
    g_side_now_state =  read_state_flags_ & 0x0F;

    /* write_state_flags_ upper bit */
    g_side_goal_reach   = (write_state_flags_ & 0x80) >> 7;
    g_side_corner_reach = (write_state_flags_ & 0x40) >> 6;
    g_side_cross_reach  = (write_state_flags_ & 0x20) >> 5;

    /* write_state_flags_ lower bit */
    g_side_black_flag  = (write_state_flags_ & 0x08) >> 3;
    g_side_goal_flag   = (write_state_flags_ & 0x04) >> 2;
    g_side_corner_flag = (write_state_flags_ & 0x02) >> 1;
    g_side_cross_flag  =  write_state_flags_ & 0x01;

    /* exception_flags_ */
    g_side_before_noise_state = (exception_flags_ & 0xF0) >> 4;
    g_side_noise_count        = (exception_flags_ & 0x0C) >> 2;
    g_side_ignore_flag        =  exception_flags_ & 0x01;
}
#endif // DEBUG_MODE