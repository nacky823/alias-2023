#include "print.hpp"
#include "macro.h"
#include <stdio.h>
#include <string.h>

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
    //int aaa = 33;
    //g_swo_test = 55;

    //printf("<<< Distance Log >>>\n");
    //printf("test%dhoge\n", aaa);
    //printf("test%fhoge\n", g_swo_test);
    //printf("<<< Distance Log 2 >>>\n");
    //Blank();
    //if(TestFlashWrite()) printf("write_success\n");
    //else printf("write_false\n");

    //DistanceLog();
    //Blank();
    printf("<<< Radian Log >>>\n");
    RadianLog();
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
    float floaty = 333.555;
    int32_t data = floaty * 100000;

    printf("floaty = %f\n", floaty);
    printf("pre_data = %d\n", data);
    printf("pre_address = %u\n", address);

    bool result = flash_->StoreInt32(address, &data, 1);

    int32_t answer = 2222;
    printf("pre_answer = %d\n", answer);

    flash_->Load(&answer, address, 4);

    printf("answer = %d\n", answer);
    printf("load_address = %u\n", address);

    float ans_floaty = answer * 0.00001;
    printf("ans_floaty = %f\n", ans_floaty);

    return result;
}

bool Print::TestIntFlash()
{
    uint32_t address = SECTOR_2_ADDRESS_HEAD;
    int16_t data = 4321;

    printf("data = %d\n", data);
    printf("pre_address = %u\n", address);

    bool result = flash_->StoreInt16(address, &data, 1);

    int16_t answer = 55;

    printf("pre_answer = %d\n", answer);

    flash_->Load(&answer, address, 2);

    printf("answer = %d\n", answer);
    printf("load_address = %u\n", address);

    return result;
}

void Print::DistanceLog()
{
    uint32_t address = SECTOR_2_ADDRESS_HEAD;
    int32_t int_data = 666666;
    float data = 66.6666;

    for(uint16_t index = 0; index < LOG_LENGTH; index++)
    {
        flash_->Load(&int_data, address, 4); // suspicious

        data = int_data * 0.00001;

        printf("%f\r\n", data);

        address += 4;
    }
}

void Print::RadianLog()
{
    uint32_t address = SECTOR_3_ADDRESS_HEAD;
    int32_t int_data = 666666;
    float data = 66.6666;

    for(uint16_t index = 0; index < LOG_LENGTH; index++)
    {
        flash_->Load(&int_data, address, 4); // suspicious

        data = int_data * 0.00001;

        //printf("%d\r\n", int_data);
        printf("%f\r\n", data);

        address += 4;
    }
}

void Print::VariousLog()
{

}

void Print::AccelPointLog()
{

}