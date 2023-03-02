#include "wrapper.hpp"
#include "run.hpp"

#ifdef TEST_MODE
Test test;
#else // TEST_MODE
Run run;
#endif // TEST_MODE

void Init()
{
#ifdef DEBUG_FLASH
    run.DubugFlash();
#else // DEBUG_FLASH
    run.Init();
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