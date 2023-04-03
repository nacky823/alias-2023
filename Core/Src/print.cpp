#include "print.hpp"
#include <stdio.h>

void Print::HelloWorld()
{
    printf("Hello, world!\r\n");
}

void Print::Log()
{
    printf("<<< Distance Log >>>\r\n");
    Blank();
    DistanceLog();
    Blank();
    printf("<<< Radian Log >>>\r\n");
    Blank();
}

void Print::Blank()
{
    for(uint8_t i = 0, i < NUM_OF_BLANK, i++)
    {
        printf("\r\n");
    }
}

void Print::DistanceLog()
{
    uint32_t address = SECTOR_2_ADDRESS_HEAD;

    for(uint16_t index = 0, index < LOG_LENGTH, index++)
    {
        flash_->Load(distance_log_[address], address, 4); // suspicious

        printf("%f\r\n", distance_log_[index]);

        address++;
    }
}

void Print::RadianLog()
{

}

void Print::VariousLog()
{

}

void Print::AccelPointLog()
{

}