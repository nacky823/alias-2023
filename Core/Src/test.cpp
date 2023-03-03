#include "test.hpp"

#ifdef TEST_MODE
Test::Test(Led *led,
           LineSensor *line_sensor)
{
    led_ = led;
    line_sensor_ = line_sensor;
}

void Test::Init()
{
    line_sensor_->Init();

    led_->Blink(3, 'Y', 'M');
    led_->ColorOrder('X');
    HAL_TIM_Base_Start_IT(&htim7);
    HAL_TIM_Base_Start_IT(&htim6);
}

void Test::Timer7()
{
    line_sensor_->StoreConsecutiveBuff();
}

void Test::Timer6()
{
    TestLineSensor();
}

void Test::TestLineSensor()
{
    line_sensor_->Update();
    line_sensor_->MonitorLineBuff();
    g_line_diff = line_sensor_->LeftRightDifference();
    g_line_emer = line_sensor_->GetEmergencyStopFlag();
    g_line_calib = line_sensor_->CheckCalibration();
    if(g_line_calib) led_->ColorOrder('B');
    else led_->ColorOrder('R');
}
#endif // TEST_MODE