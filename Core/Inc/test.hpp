#ifndef TEST_HPP_
#define TEST_HPP_

#include "led.hpp"
#include "line_sensor.hpp"
#include "side_sensor.hpp"

#ifdef TEST_MODE
class Test
{
private:
    Led *led_;
    LineSensor *line_sensor_;
    SideSensor *side_sensor_;

public:
    Test(Led *,
         LineSensor *,
         SideSensor *);
    void Init();
    void Timer7();
    void Timer6();
    void TestLineSensor();
    void TestSideSensor();

};
#endif // TEST_MODE

#endif // TEST_HPP_