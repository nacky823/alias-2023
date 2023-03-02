#include "wrapper.hpp"
#include "test.hpp"
#include "run.hpp"

#ifdef TEST_MODE
Test test;
#endif // TEST_MODE
Run run;

void Init()
{
#ifdef TEST_HPP_
    test.Init();
#else // TEST_HPP_
    run.Init();
#endif // TEST_HPP_
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