#include "plot_smoothing.hpp"
#include <math.h>

PlotSmoothing::PlotSmoothing
: now_address_(0)
, radian_stack_(0)
{

}

void PlotSmoothing::SetNowAddress(uint16_t address)
{
    now_address_ = address;
}

void PlotSmoothing::ResetRadianAddress()
{
    SetRadianAddress(SECTOR_3_ADDRESS_HEAD);
}

void PlotSmoothing::SetRadianAddress(uint32_t address)
{
    now_address_ = address;
}

uint32_t PlotSmoothing::GetRadianAddress()
{
    return radian_address_;
}

void PlotSmoothing::StackRadian()
{
    uint32_t address = radian_address_;
    uint32_t int_data = 555555;
    float radian = flash_->Load(&int_data, address, 4);

    radian_stack_ += radian;
    radian_address_ += 4;
}

void PlotSmoothing::SetDistance(float distance)
{
    distance_ = distance;
}

void PlotSmoothing::StoreDistance()
{
    uint32_t relative_flash_address = now_address_ * FLOAT_SIZE;
    uint32_t absolute_flash_address = SECTOR_2_ADDRESS_HEAD + relative_flash_address;
    uint32_t int_data = INITIAL_INT_DATA;
    float distance = flash_->Load(&int_data, absolute_flash_address, FLOAT_SIZE);

    SetDistance(distance);

    uint16_t next_address = now_address_ + 1;
    SetNowAddress(next_address);
}

void PlotSmoothing::CalculateCoordinate()
{
    double distance = distance_;
    double radian = radian_;

    double y = distance * sin(radian) + y_coordinate_;
    y_coordinate_ = y;
    double x = distance * cos(radian) + x_coordinate_;
    x_coordinate_ = x;
}

void PlotSmoothing::Smoothing(float *data_addr, uint16_t data_size, uint16_t num_of_adjacent)
{
    float cp_data[data_size];
    uint16_t i, j, max;
    int16_t min;
    double sum;
    float avg;

    for(i = 0; i < data_size; i++) cp_data[i] = 0;

    for(i = 0; i < data_size; i++)
    {
        sum = 0;
        min = i - num_of_adjacent;
        max = i + num_of_adjacent;
        for(j = min; j <= max; j++)
        {
            if(j < 0 || j >= data_size) continue;
            sum += *(data_addr + j);
        }
        if(i < num_of_adjacent || i >= (data_size - num_of_adjacent))
        {
            cp_data[i] = *(data_addr + i);
            continue;
        }
        avg = (float)sum / (num_of_adjacent * 2 + 1);
        cp_data[i] = avg;
    }

    for(i = 0; i < data_size; i++)
    {
        *(data_addr + i) = cp_data[i];
    }
}