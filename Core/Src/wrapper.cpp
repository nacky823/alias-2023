#include "wrapper.hpp"
#include "test.hpp"
#include "run.hpp"

Encoder encoder;
Flash flash;
Iim42652 iim_42652;
Imu imu(&iim_42652);
Led led;
LineSensor line_sensor;
LineTrace line_trace(&line_sensor);
Motor motor;
Print print(&flash);
RotarySwitch rotary_switch;
SideSensor side_sensor;
VelocityControl velocity_control(&encoder);
Logger logger(&encoder, &flash, &led, &imu, &side_sensor);

#ifndef TEST_MODE
Run run(
    &encoder,
    &flash,
    &iim_42652,
    &imu,
    &led,
    &line_sensor,
    &line_trace,
    &logger,
    &motor,
    &print,
    &rotary_switch,
    &side_sensor,
    &velocity_control
);
#else // TEST_MODE
Test test(
    &encoder,
    &iim_42652,
    &led,
    &line_sensor,
    &line_trace,
    &motor,
    &rotary_switch,
    &side_sensor,
    &velocity_control
);
#endif // TEST_MODE

void Init()
{
#ifndef TEST_MODE
    run.Init();
#else // TEST_MODE
    test.Init();
#endif // TEST_MODE
}

void InterruptTim7()
{
#ifndef TEST_MODE
    run.Timer7();
#else // TEST_MODE
    test.Timer7();
#endif // TEST_MODE

#ifdef DEBUG_MODE
    g_tim7++;
#endif // DEBUG_MODE
}

void InterruptTim6()
{
#ifndef TEST_MODE
    run.RunMode();
#else // TEST_MODE
    test.Timer6();
#endif // TEST_MODE

#ifdef DEBUG_MODE
    g_tim6++;
#endif // DEBUG_MODE
}

void InterruptTim2()
{
#ifndef TEST_MODE
    run.Timer2();
#endif // TEST_MODE

#ifdef DEBUG_MODE
    g_tim2++;
#endif // DEBUG_MODE
}

#ifdef DEBUG_MODE
void RamLog()
{
    print_->RamLog();
}
#endif // DEBUG_MODE