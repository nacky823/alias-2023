#include "rotary_switch.hpp"

uint8_t RotarySwitch::State()
{
    uint8_t switch_state = 0x00;

    if(PIN_1 == 0) switch_state |= 0x01;
    if(PIN_2 == 0) switch_state |= 0x02;
    if(PIN_4 == 0) switch_state |= 0x04;
    if(PIN_8 == 0) switch_state |= 0x08;

    return switch_state;
}