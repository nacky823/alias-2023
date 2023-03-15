#ifndef TEST_HPP_
#define TEST_HPP_

#include "encoder.hpp"
#include "led.hpp"
#include "line_sensor.hpp"
#include "line_trace.hpp"
#include "motor.hpp"
#include "rotary_switch.hpp"
#include "side_sensor.hpp"
#include "velocity_control.hpp"

#ifdef TEST_MODE
class Test
{
private:
    Encoder *encoder_;
    Led *led_;
    LineSensor *line_sensor_;
    LineTrace *line_trace_;
    Motor *motor_;
    RotarySwitch *rotary_switch_;
    SideSensor *side_sensor_;
    VelocityControl *velocity_control_;

public:
    Test(Encoder *,
         Led *,
         LineSensor *,
         LineTrace *,
         Motor *,
         RotarySwitch *,
         SideSensor *,
         VelocityControl *);
    void Init();
    void Timer7();
    void Timer6();
    void TestLineSensor();
    void TestSideSensor();
    void TestMotor();
    void TestLineTrace();
    void TestEncoder();
    void TestVelocityControl();
    void RunLineTrace();
    void MonitorLog();

};
#endif // TEST_MODE

#endif // TEST_HPP_