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

#define LOGGING_CONST_DISTANCE 20.0
#define MAX_LOG_INDEX 80

#ifdef DEBUG_MODE
extern float g_velocity;
extern float g_angular_velocity;
extern float g_distance_log[MAX_LOG_INDEX];
extern float g_angular_log[MAX_LOG_INDEX];
#endif // DEBUG_MODE

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