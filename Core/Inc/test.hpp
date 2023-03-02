#ifndef TEST_HPP_
#define TEST_HPP_

class Test
{
private:
    LineSensor *line_sensor_;

public:
    Test(LineSensor *);
    void Init();
    void Timer6();
    void LineSensor();

};

#endif // TEST_HPP_