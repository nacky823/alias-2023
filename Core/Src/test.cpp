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

    if(rotary_switch_->State() != 0x0C)
    {
        HAL_TIM_Base_Start_IT(&htim7);
        HAL_TIM_Base_Start_IT(&htim6);
    }
    else led_->Blink(3, 'B', 'X');
}

void Test::Loop()
{
    printf("Hello,World!!\r\n");
}

void Test::Timer7()
{
    line_sensor_->StoreConsecutiveBuff();
}

void Test::Timer6()
{
    TestLineSensor();
    TestSideSensor();
    TestEncoder();
    MonitorLog();
    TestMotor();
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
        case 0x0C: RunVelocityControl();  break;
        case 0x0D: 
            g_goal_count = 0;
            motor_->Drive(0, 0);
            break;
        case 0x0E: TestVelocityControl(); break;
        case 0x0F: TestLineTrace();       break;
        case 0x00: motor_->Drive(0.2, 0);   break;
        case 0x01: motor_->Drive(-0.2, 0);  break;
        case 0x03: motor_->Drive(1.0, 0);   break;
        case 0x04: motor_->Drive(0.5, 0.6); break;
        default:   motor_->Drive(0, 0);     break;
    }
}

void Test::TestLineTrace()
{
    if(g_goal_count < 2)
    {
        float rotat = line_trace_->LineTraceOnly();

        motor_->Drive(0, rotat);
    }
    else
    {
        led_->ColorOrder('Y');
        motor_->Drive(0 ,0);
    }
}

void Test::TestEncoder()
{
    encoder_->Update();
    g_velocity = (encoder_->GetDistance()) / TIM6_PERIOD;
    g_angular_velocity = encoder_->AngularVelocity();
}

void Test::TestVelocityControl()
{
    float trans = velocity_control_->DeterminePidGain(1.0);

    motor_->Drive(trans, 0);
}

void Test::MonitorLog()
{
    float distance = encoder_->GetDistanceStack();

    if(distance < LOGGING_CONST_DISTANCE) return;
    encoder_->ResetDistanceStack();

    static uint16_t log_index = 0;   // Count up when interrupt.

    g_distance_log[log_index] = distance;
    g_angular_log[log_index] = encoder_->AngularVelocity();

    log_index++;
    if(log_index >= MAX_LOG_INDEX) log_index = 0;
}

void Test::RunVelocityControl()
{
    static uint8_t stop_count = 0;
    bool emergency = line_sensor_->GetEmergencyStopFlag();

    if(emergency)
    {
        stop_count++;
    }
    else stop_count = 0;

    if(stop_count >= 5)
    {
        motor_->Drive(0, 0);

        led_->ColorOrder('C');

        g_goal_count = 2;

        return;
    }

    if(g_goal_count < 2)
    {
        float trans = velocity_control_->DeterminePidGain(1.0);
        float rotat = line_trace_->LineTraceOnly();

        motor_->Drive(trans, rotat);
    }
    else
    {
        float trans = velocity_control_->DeterminePidGain(0.0);

        motor_->Drive(trans, 0);

        led_->ColorOrder('M');
    }
}
#endif // TEST_MODE