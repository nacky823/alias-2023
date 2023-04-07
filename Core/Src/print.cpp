#include "print.hpp"
#include "macro.h"
#include <stdio.h>

extern float g_swo_test;

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
    int aaa = 33;
    g_swo_test = 55;

    printf("<<< Distance Log >>>\n");
    //printf("test%dhoge\n", aaa);
    //printf("test%fhoge\n", g_swo_test);
    //printf("<<< Distance Log 2 >>>\n");
    //Blank();
    if(TestFlashWrite()) printf("write_success\n");
    else printf("write_false\n");
    DistanceLog();
    //Blank();
    printf("<<< Radian Log >>>\n");
    //Blank();
}

void Print::Blank()
{
    for(uint8_t i = 0; i < NUM_OF_BLANK; i++)
    {
        printf("\r\n");
    }
}

bool Print::TestFlashWrite()
{
    uint32_t address = SECTOR_2_ADDRESS_HEAD;
    float data = 333.555;

    //printf("pre_data = %f\n", data);
    printf("pre_address = %u\n", address);

    bool result = flash_->StoreFloat(address, &data, 1);
    //printf("data = %f\n", data);
    printf("address = %u\n", address);

    float answer = 22;
    printf("pre_answer = %f\n", answer);

    flash_->Load(&answer, address, 4);
    printf("load_address = %u\n", address);

    printf("answer = %f\n", answer);

    return result;
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