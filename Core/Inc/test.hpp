#ifndef TEST_HPP_
#define TEST_HPP_

#include "led.hpp"
#include "line_sensor.hpp"
#include "motor.hpp"
#include "rotary_switch.hpp"
#include "side_sensor.hpp"

#ifdef TEST_MODE
class Test
{
private:
    Led *led_;
    LineSensor *line_sensor_;
    Motor *motor_;
    RotarySwitch *rotary_switch_;
    SideSensor *side_sensor_;

public:
    Test(Led *,
         LineSensor *,
         Motor *,
         RotarySwitch *,
         SideSensor *);
    void Init();
    void Timer7();
    void Timer6();
    void TestLineSensor();
    void TestSideSensor();
    void TestMotor();

};
#endif // TEST_MODE

#endif // TEST_HPP_