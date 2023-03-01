#include "led.hpp"

void Led::ColorOrder(char color)
{
    switch(color)
    {
        case 'R': R_ON;  G_OFF; B_OFF; break;
        case 'G': R_OFF; G_ON;  B_OFF; break;
        case 'B': R_OFF; G_OFF; B_ON;  break;
        case 'Y': R_ON;  G_ON;  B_OFF; break;
        case 'C': R_OFF; G_ON;  B_ON;  break;
        case 'M': R_ON;  G_OFF; B_ON;  break;
        case 'W': R_ON;  G_ON;  B_ON;  break;
        case 'X': // fall through
        default:  R_OFF; G_OFF; B_OFF; break;
    }
}

void Led::Blink(uint8_t times, char color_1, char color_2)
{
    for(uint8_t i = 0; i < times; i++)
    {
        ColorOrder(color_1);
        HAL_Delay(BLINK_INTERVAL_MS);
        ColorOrder(color_2);
        HAL_Delay(BLINK_INTERVAL_MS);
    }
}

void Led::Rainbow(uint8_t times)
{
    for(uint8_t i = 0; i < times; i++)
    {
        ColorOrder('R');
        HAL_Delay(RAINBOW_INTERVAL_MS);
        ColorOrder('Y');
        HAL_Delay(RAINBOW_INTERVAL_MS);
        ColorOrder('G');
        HAL_Delay(RAINBOW_INTERVAL_MS);
        ColorOrder('C');
        HAL_Delay(RAINBOW_INTERVAL_MS);
        ColorOrder('B');
        HAL_Delay(RAINBOW_INTERVAL_MS);
        ColorOrder('M');
        HAL_Delay(RAINBOW_INTERVAL_MS);
    }
}

bool Led::BlinkInterrupt(uint8_t times, char color_1, char color_2)
{
    uint8_t count = interrupt_count_;
    uint8_t timer = interrupt_timer_;
    bool blink_continue = true;

    if(count < times){
        if(timer < BLINK_INTERVAL_TIMX){
            ColorOrder(color_1);
            timer++;
        }else if(timer < (BLINK_INTERVAL_TIMX + BLINK_INTERVAL_TIMX)){
            ColorOrder(color_2);
            timer++;
        }else{
            count++;
            timer = 0;
        }
    }else{
        ColorOrder('X');
        count = 0;
        blink_continue = false;
    }

    interrupt_count_ = count;
    interrupt_timer_ = timer;

    return blink_continue;
}

void Led::ResetInterrupt()
{
    interrupt_count_ = 0;
    interrupt_timer_ = 0;
}