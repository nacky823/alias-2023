#include "test.hpp"

Test::Test(LineSensor *line_sensor)
{
    line_sensor_ = line_sensor;
}

void Test::Init()
{
    line_sensor_->Init();
    encoder_->Init();
    motor_->Init();

    led_->Rainbow(2);
    HAL_TIM_Base_Start_IT(&htim6);
    HAL_TIM_Base_Start_IT(&htim7);
}

void Test::Timer6()
{
    LineSensor();
}

void Test::LineSensor()
{
    line_sensor_->Update();
    line_sensor_->MonitorArrays();
    g_line_diff = line_sensor_->LeftRightDifference();
    g_line_emer = line_sensor_->GetEmergencyStopFlag();
    g_line_calib = line_sensor_->CheckCalibration();
}