#ifdef TEST_MODE
#ifndef TEST_HPP_
#define TEST_HPP_

#include "line_senser.hpp"
#include "led.hpp"

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

#endif // TEST_HPP_
#endif // TEST_MODE