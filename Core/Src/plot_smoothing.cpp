#include "plot_smoothing.hpp"
#include "macro.h"
#include <stdio.h>
#include <math.h>

#define FLOAT_SIZE 4

PlotSmoothing::PlotSmoothing(Flash *flash)
: radian_(0)
, distance_(0)
, x_coordinate_(0)
, y_coordinate_(0)
, x_()
, y_()
{
    flash_ = flash;
}

void PlotSmoothing::SetRadian(float radian)
{
    radian_ = radian;
}

void PlotSmoothing::StackRadian(uint16_t count)
{
    uint32_t relative_flash_address = count * FLOAT_SIZE;
    uint32_t absolute_flash_address = SECTOR_3_ADDRESS_HEAD + relative_flash_address;
    uint32_t int_radian = INITIAL_INT_DATA;
    flash_->Load(&int_radian, absolute_flash_address, FLOAT_SIZE);
    float radian = int_radian * 0.00001;

    float radian_stack = radian_ + radian;
    SetDistance(radian_stack);
}

void PlotSmoothing::SetDistance(float distance)
{
    distance_ = distance;
}

void PlotSmoothing::StoreDistance(uint16_t count)
{
    uint32_t relative_flash_address = count * FLOAT_SIZE;
    uint32_t absolute_flash_address = SECTOR_2_ADDRESS_HEAD + relative_flash_address;
    uint32_t int_distance = INITIAL_INT_DATA;
    flash_->Load(&int_distance, absolute_flash_address, FLOAT_SIZE);
    float distance = int_distance * 0.00001;

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
        StoreDistance(i);
        StackRadian(i);
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
            sum += *(FLOAT_SIZE * j + data_addr);
        }
        if(i < num_of_adjacent || i >= (data_size - num_of_adjacent))
        {
            cp_data[i] = *(FLOAT_SIZE * i + data_addr);
            continue;
        }
        avg = (float)sum / (num_of_adjacent * 2 + 1);
        cp_data[i] = avg;
    }

    for(i = 0; i < data_size; i++)
    {
        *(FLOAT_SIZE * i + data_addr) = cp_data[i];
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
    float *x_ptr = &x_[0];
    float *y_ptr = &y_[0];

    StoreCoordinate();
    Smoothing(x_ptr, MAX_LOG, 2);
    Smoothing(y_ptr, MAX_LOG, 2);
    Print();
}