#include "print.hpp"
#include "macro.h"
#include <stdio.h>

Print::Print(Flash *flash)
{
    flash_ = flash;
}

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
    for(uint8_t i = 0; i < NUM_OF_BLANK; i++)
    {
        printf("\r\n");
    }
}

void Print::DistanceLog()
{
    uint32_t address = SECTOR_2_ADDRESS_HEAD;
    float data;

    for(uint16_t index = 0; index < LOG_LENGTH; index++)
    {
        flash_->Load(&data, address, 4); // suspicious

        printf("%f\r\n", data);

        address += 4;
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