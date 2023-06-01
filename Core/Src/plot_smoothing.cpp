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

uint16_t PlotSmoothing::GetNowAddress()
{
    return now_address_;
}

void PlotSmoothing::SetRadian(float radian)
{
    radian_ = radian;
}

void PlotSmoothing::StackRadian(uint16_t count)
{
    uint32_t relative_flash_address = now_address_ * FLOAT_SIZE;
    uint32_t absolute_flash_address = SECTOR_3_ADDRESS_HEAD + relative_flash_address;
    uint32_t int_data = INITIAL_INT_DATA;
    float radian = flash_->Load(&int_data, absolute_flash_address, FLOAT_SIZE);

    float radian_stack = radian_ + radian;
    SetDistance(radian_stack);

    uint16_t next_address = now_address_ + 1;
    SetNowAddress(next_address);
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

void PlotSmoothing::StoreCoordinate()
{
    for(uint16_t i = 0; i < MAX_LOG; i++)
    {
        StoreDistance();
        StackRadian();
        CalculateCoordinate();
        x_[i] = x_coordinate_;
        y_[i] = y_coordinate_;
    }
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

void PlotSmoothing::Print()
{
    printf("<<< x coordinate >>>\n");
    for(uint16_t i = 0; i < MAX_LOG; i++) printf("%f\n", x_[i]);

    printf("<<< y coordinate >>>\n");
    for(uint16_t i = 0; i < MAX_LOG; i++) printf("%f\n", y_[i]);
}

void PlotSmoothing::Run()
{
    StoreCoordinate();
    Smoothing();
    Print();
}