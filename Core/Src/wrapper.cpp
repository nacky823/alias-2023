#include "wrapper.hpp"
#include "run.hpp"

Run run;

void Init()
{
#ifndef DEBUG_FLASH
    run.Init();
#else // DEBUG_FLASH
    run.DubugFlash();
#endif // DEBUG_FLASH
}

void InterruptTim7()
{
    run.Timer7();

#ifdef DEBUG_MODE
    g_tim7++;
#endif // DEBUG_MODE
}

void InterruptTim6()
{
    run.RunMode();

#ifdef DEBUG_MODE
    g_tim6++;
#endif // DEBUG_MODE
}

void InterruptTim2()
{
    run.UpdateRunMode();

#ifdef DEBUG_MODE
    g_tim2++;
#endif // DEBUG_MODE
}