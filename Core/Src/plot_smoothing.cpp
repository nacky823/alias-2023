#include "plot_smoothing.hpp"

PlotSmoothing::PlotSmoothing
: now_address_(0)
, radian_stack_(0)
{

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