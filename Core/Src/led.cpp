#include "led.hpp"

void Led::ColorOrder(char color)
{
    switch(color)
    {
        case 'R': R_ON;  G_OFF; B_OFF; break;
        case 'G': R_OFF; G_ON;  B_OFF; break;
        case 'B': R_OFF; G_OFF; B_ON;  break;
        case 'C': R_OFF; G_ON;  B_ON;  break;
        case 'M': R_ON;  G_OFF; B_ON;  break;
        case 'Y': R_ON;  G_ON;  B_OFF; break;
        case 'W': R_ON;  G_ON;  B_ON;  break;
        case 'X': // fall through
        default:  R_OFF; G_OFF; B_OFF; break;
    }
}