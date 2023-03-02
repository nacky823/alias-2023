#ifndef TEST_HPP_
#define TEST_HPP_

#include "line_sensor.hpp"
#include "led.hpp"

#ifdef TEST_MODE
class Test
{
private:
    LineSensor *line_sensor_;
    Led *led_;

public:
    Test(LineSensor *,
         Led *);
    void Init();
    void Timer7();
    void Timer6();
    void LineSensor();

};
#endif // TEST_MODE

#endif // TEST_HPP_