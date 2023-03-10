#include "test.hpp"

#ifdef TEST_MODE
Test::Test(Encoder *encoder,
           Led *led,
           LineSensor *line_sensor,
           LineTrace *line_trace,
           Motor *motor,
           RotarySwitch *rotary_switch,
           SideSensor *side_sensor,
           VelocityControl *velocity_control)
{
    encoder_ = encoder;
    led_ = led;
    line_sensor_ = line_sensor;
    line_trace_ = line_trace;
    motor_ = motor;
    rotary_switch_ = rotary_switch;
    side_sensor_ = side_sensor;
    velocity_control_ = velocity_control;
}

void Test::Init()
{
    encoder_->Init();
    line_sensor_->Init();
    motor_->Init();

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
    TestSideSensor();
    TestMotor();
    TestEncoder();
}

void Test::TestLineSensor()
{
    line_sensor_->Update();
    line_sensor_->MonitorLineBuff();
    g_line_diff = line_sensor_->LeftRightDifference();
    g_line_emer = line_sensor_->GetEmergencyStopFlag();
    g_line_calib = line_sensor_->CheckCalibration();
    if(g_line_calib) led_->ColorOrder('X');
    else led_->ColorOrder('R');
}

void Test::TestSideSensor()
{
    side_sensor_->Update();
    side_sensor_->Monitor();
}

void Test::TestMotor()
{
    g_switch_state = rotary_switch_->State();

    switch(g_switch_state)
    {
        case 0x0F: TestLineTrace(); break;
        case 0x00: motor_->Drive(0.2, 0);   break;
        case 0x01: motor_->Drive(-0.2, 0);  break;
        case 0x03: motor_->Drive(1.0, 0);   break;
        case 0x04: motor_->Drive(0.5, 0.6); break;
        default:   motor_->Drive(0, 0);     break;
    }
}

void Test::TestLineTrace()
{
    float rotat = line_trace_->LineTraceOnly();

    motor_->Drive(0, rotat);
}

void Test::TestEncoder()
{
    encoder_->Update();
}

void Test::TestVelocityControl()
{
    float trans = 


}
#endif // TEST_MODE