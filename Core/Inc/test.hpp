#ifndef TEST_HPP_
#define TEST_HPP_

#include "line_sensor.hpp"
#include "led.hpp"

#ifdef TEST_MODE
class Test
{
private:
    Led *led_;
    LineSensor *line_sensor_;

public:
    Test(Led *,
         LineSensor *);
    void Init();
    void Timer7();
    void Timer6();
    void TestLineSensor();

};
#endif // TEST_MODE

#endif // TEST_HPP_