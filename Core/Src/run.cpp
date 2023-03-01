#include "run.hpp"
#include "declare_extern.h"

Run::Run(Encoder *encoder,
         Flash *flash,
         Iim42652 *iim_42652,
         Led *led,
         LineSensor *line_sensor,
         LineTrace *line_trace,
         Logger *logger,
         Moter *motor,
         RotarySwitch *rotary_switch,
         SideSensor *side_sensor,
         VelocityControl *velocity_control
         ) : run_mode_(0)
           , switch_interval_wait_timer_(0)
           , switch_interval_wait_enable_(true)
           , switch_interval_led_enable_(true)
           , mode_complete_(true)
           , store_log_failed_(true)
{
    Encoder_         = Encoder;
    Flash_           = flash;
    Iim42652_        = iim_42652;
    Led_             = led;
    LineSensor_      = line_sensor;
    LineTrace_       = line_trace;
    Logger_          = logger;
    Motor_           = motor;
    RotarySwitch_    = rotary_switch;
    SideSensor_      = side_sensor;
    VelocityControl_ = velocity_control;
}

void Run::Init()
{
    line_sensor.Init();
    encoder.Init();
    motor.Init();
    uint8_t imu_init = iim_42652.Init();

    bool flash_erase = true;
    uint8_t switch_state = rotary_switch.State();
    if(switch_state == 0x0E)
    {
        led.Blink(3, 'R', 'X');
        if(!flash.Clear()) flash_erase = false;
    }

#ifdef DEBUG_MODE
    g_imu_init = imu_init;
    g_flash_erase = flash_erase;
    g_switch_state = switch_state;
#endif // DEBUG_MODE

    if(flash_erase && imu_init == 0x09)
    {
        led.Rainbow(1);
        HAL_TIM_Base_Start_IT(&htim2);
        HAL_TIM_Base_Start_IT(&htim6);
        HAL_TIM_Base_Start_IT(&htim7);
    }
    else if(flash_erase) led.ColorOrder('Y');
    else led.ColorOrder('R');
}

void Run::UpdateRunMode()
{
    uint8_t switch_state = rotary_switch.State();
    static uint8_t pre_switch_state = switch_state;
    static bool interval_enable = true;

#ifdef DEBUG_MODE
    g_switch_state = switch_state;
    g_interval_enable = interval_enable;
#endif // DEBUG_MODE

    if(switch_state != pre_switch_state)
    {
        SetRunMode(STANDBY);
        WhenSwitchChange();
        led.ResetInterrupt();
        interval_enable = true;
    }
    else if(interval_enable)
    {
        SetRunMode(STANDBY);
        interval_enable = SwitchChangeInterval(switch_state);
    }
    else
    {
        switch(switch_state)
        {
#ifdef DEBUG_MODE
            case 0x0B: SetRunMode(VELOCITY_CONTROL_DEBUG); break;
            case 0x0C: SetRunMode(LINE_TRACE_DEBUG); break;
            case 0x0D: SetRunMode(INITIAL_DEBUG); break;
#endif // DEBUG_MODE

            case 0x0F: SetRunMode(READY); break;
            case 0x01: SetRunMode(FIRST_RUN); break;
            case 0x02: SetRunMode(SECOND_RUN); break;
            case 0x07: SetRunMode(VELOCITY_CONTROL); break;
            case 0x08: SetRunMode(LINE_TRACE); break;
            default:   SetRunMode(STANDBY); break;
        }
    }

    pre_switch_state = switch_state;
}

void Run::SetRunMode(uint8_t mode)
{
    run_mode_ = mode;
}

void Run::WhenSwitchChange()
{
    switch_interval_led_enable_ = true;
    switch_interval_wait_enable_ = true;
    switch_interval_wait_timer_ = 0;
}

bool Run::SwitchChangeInterval(uint8_t switch_state)
{
    uint8_t wait_timer = switch_interval_wait_timer_;
    bool wait_enable = switch_interval_wait_enable_;
    bool led_enable = switch_interval_led_enable_;
    bool interval_continue = true;

    if(led_enable)
    {
        switch(switch_state)
        {
#ifdef DEBUG_MODE
            case 0x0B: led_enable = led.BlinkInterrupt(3, 'G', 'B'); break;
            case 0x0C: led_enable = led.BlinkInterrupt(3, 'G', 'Y'); break;
            case 0x0D: led_enable = led.BlinkInterrupt(3, 'G', 'X'); break;
#endif // DEBUG_MODE

            case 0x0F: led_enable = led.BlinkInterrupt(3, 'Y', 'X'); break;
            case 0x01: led_enable = led.BlinkInterrupt(3, 'B', 'X'); break;
            case 0x02: led_enable = led.BlinkInterrupt(3, 'M', 'X'); break;
            case 0x07: led_enable = led.BlinkInterrupt(3, 'G', 'B'); break;
            case 0x08: led_enable = led.BlinkInterrupt(3, 'G', 'Y'); break;
            default:   led_enable = led.BlinkInterrupt(3, 'X', 'W'); break;
        }
    }
    else if(wait_enable)
    {
        if(wait_timer < SWITCH_INTERVAL_WAIT_TIME)
        {
            wait_timer++;
            wait_enable = true;
        }
        else wait_enable = false;
    }
    else interval_continue = false;
    
    switch_interval_led_enable_ = led_enable;
    switch_interval_wait_enable_ = wait_enable;
    switch_interval_wait_timer_ = wait_timer;

    return interval_continue;
}

bool Run::EmergencyStop()
{
    static uint8_t emergency_timer = 0;
    bool line_emergency = line_sensor.GetEmergencyStopFlag();

    if(emergency_timer >= EMERGENCY_STOP_TIME)
    {
        SetRunMode(EMERGENCY);
        return true;
    }
    else if(line_emergency) emergency_timer++;
    else emergency_timer = 0;

    return false;
}

void Run::RunMode()
{
    switch(run_mode_)
    {
#ifdef DEBUG_MODE
        case VELOCITY_CONTROL_DEBUG: ModeVelocityControlDebug(); break;
        case LINE_TRACE_DEBUG: ModeLineTraceDebug(); break;
        case INITIAL_DEBUG: ModeInitialDebug(); break;
#endif // DEBUG_MODE

        case EMERGENCY: ModeEmergency(); break;
        case READY: ModeReady(); break;
        case STANDBY: ModeStandby(); break;
        case FIRST_RUN: ModeFirstRun(); break;
        case FIRST_GOAL: ModeFirstGoal(); break;
        case SECOND_RUN: ModeSecondRun(); break;
        case SECOND_GOAL: ModeSecondGoal(); break;
        case VELOCITY_CONTROL: ModeVelocityControl(); break;
        case LINE_TRACE: ModeLineTrace(); break;
        default: ModeStandby(); break;
    }

    mode_complete_ = true;
}

void Run::ModeEmergency()
{
    motor.Drive(0, 0);
}

void Run::ModeReady()
{
    line_sensor.Update();
    if(line_sensor.CheckCalibration()) led.ColorOrder('X');
    else led.ColorOrder('R');
}

void Run::ModeStandby()
{
    motor.Drive(0, 0);
}

void Run::ModeFirstRun()
{
    /* Sensor update */
    line_sensor.Update();
    encoder.Update();
    iim_42652.Update();
    side_sensor.IgnoreJudgment();
    /* Emergency stop */
    if(EmergencyStop()) return;
    /* Logging */
    uint8_t goal_count = side_sensor.GetGoalMarkerCount();
    if(goal_count == 1) logger.Logging(mode_complete_);
    mode_complete_ = false;
    uint8_t period_success = logger.StorePeriodicLog();
    uint8_t accel_success = logger.StoreAccelPositionLog();
    if(period_success != 0 || !accel_success != 0) store_log_failed_ = true;
    /* Motor control */
    float target_velocity = FirstTargetVelocity(goal_count);
    float trans_ratio = velocity_control.DeterminePidGain(target_velocity);
    float rotat_ratio = line_trace.DeterminePidGain(target_velocity);
    motor.Drive(trans_ratio, rotat_ratio);
}

void Run::ModeFirstGoal()
{
    motor.Drive(0, 0);
    if(store_log_failed_) led.ColorOrder('R');
    else led.ColorOrder('B');
}

void Run::ModeSecondRun()
{
    /* Sensor update */
    line_sensor.Update();
    encoder.Update();
    iim_42652.Update();
    side_sensor.IgnoreJudgment();
    /* Emergency stop */
    if(EmergencyStop()) return;
    /* Loading */
    uint8_t goal_count = side_sensor.GetGoalMarkerCount();
    if(goal_count == 1) logger.Loading();
    /* Motor control */
    float target_velocity = SecondTargetVelocity(goal_count);
    float trans_ratio = velocity_control.DeterminePidGain(target_velocity);
    float rotat_ratio = line_trace.DeterminePidGain(target_velocity);
    motor.Drive(trans_ratio, rotat_ratio);
}

void Run::ModeSecondGoal()
{
    motor.Drive(0, 0);
    led.ColorOrder('M');
}

void Run::ModeVelocityControl()
{
    /* Sensor update */
    line_sensor.Update();
    encoder.Update();
    side_sensor.IgnoreJudgment();
    /* Emergency stop */
    if(EmergencyStop()) return;
    /* Motor control */
    uint8_t goal_count = side_sensor.GetGoalMarkerCount();
    float target_velocity = VelocityControlTarget(goal_count);
    float trans_ratio = velocity_control.DeterminePidGain(target_velocity);
    float rotat_ratio = line_trace.DeterminePidGain(target_velocity);
    motor.Drive(trans_ratio, rotat_ratio);
}

void Run::ModeLineTrace()
{
    /* Sensor update */
    line_sensor.Update();
    side_sensor.IgnoreJudgment();
    /* Emergency stop */
    if(EmergencyStop()) return;
    /* Motor control */
    uint8_t goal_count = side_sensor.GetGoalMarkerCount();
    float trans_ratio = CommonDuty(goal_count);
    float rotat_ratio = line_trace.LineTraceOnly();
    motor.Drive(trans_ratio, rotat_ratio);
}

#ifdef DEBUG_MODE
void Run::ModeVelocityControlDebug()
{
    /* Sensor update */
    line_sensor.Update();
    encoder.Update();
    /* Motor control */
    float target_velocity = VELOCITY_CONTROL_TARGET;
    float trans_ratio = velocity_control.DeterminePidGain(target_velocity);
    motor.Drive(trans_ratio, 0);
}

void Run::ModeLineTraceDebug()
{
    /* Sensor update */
    line_sensor.Update();
    /* Motor control */
    float rotat_ratio = line_trace.LineTraceOnly();
    motor.Drive(0, rotat_ratio);
}

void Run::ModeInitialDebug()
{
    InitialTest();
}
#endif // DEBUG_MODE

float Run::FirstTargetVelocity(uint8_t goal_count)
{
    static uint8_t slow_timer = 0;
    static uint8_t stop_timer = 0;

    if(goal_count == 1)
    {
        return EXPLORE_VELOCITY;
    }
    else if(goal_count >= 2)
    {
        if(slow_timer < SLOW_DRIVE_TIME)
        {
            slow_timer++;
            return SLOW_DRIVE_VELOCITY;
        }
        else if(stop_timer < STOP_TIME)
        {
            stop_timer++;
            return 0;
        }
        else
        {
            SetRunMode(FIRST_GOAL);
            return 0;
        }
    }
    else return EXPLORE_VELOCITY;
}

float Run::SecondTargetVelocity(uint8_t goal_count)
{
    static uint8_t slow_timer = 0;
    static uint8_t stop_timer = 0;

    if(goal_count == 1)
    {
        return logger.GetTargetVelocity();
    }
    else if(goal_count >= 2)
    {
        if(slow_timer < SLOW_DRIVE_TIME)
        {
            slow_timer++;
            return SLOW_DRIVE_VELOCITY;
        }
        else if(stop_timer < STOP_TIME)
        {
            stop_timer++;
            return 0;
        }
        else
        {
            SetRunMode(SECOND_GOAL);
            return 0;
        }
    }
    else return MIN_VELOCITY;
}

float Run::VelocityControlTarget(uint8_t goal_count)
{
    static uint8_t slow_timer = 0;
    static uint8_t stop_timer = 0;

    if(goal_count == 1)
    {
        return VELOCITY_CONTROL_TARGET;
    }
    else if(goal_count >= 2)
    {
        if(slow_timer < SLOW_DRIVE_TIME)
        {
            slow_timer++;
            return SLOW_DRIVE_VELOCITY;
        }
        else if(stop_timer < STOP_TIME)
        {
            stop_timer++;
            return 0;
        }
        else
        {
            SetRunMode(STANDBY);
            return 0;
        }
    }
    else return VELOCITY_CONTROL_TARGET;
}

float Run::CommonDuty(uint8_t goal_count)
{
    static uint8_t slow_timer = 0;
    static uint8_t stop_timer = 0;

    if(goal_count == 1)
    {
        return COMMON_DUTY;
    }
    else if(goal_count >= 2)
    {
        if(slow_timer < SLOW_DRIVE_TIME)
        {
            slow_timer++;
            return SLOW_DRIVE_DUTY;
        }
        else if(stop_timer < STOP_TIME)
        {
            stop_timer++;
            return 0;
        }
        else
        {
            SetRunMode(STANDBY);
            return 0;
        }
    }
    else return COMMON_DUTY;
}

#ifdef DEBUG_FLASH
void Run::DubugFlash()
{
    if(rotary_switch.State() == 0x0A)
    {
        led.Blink(5, 'R', 'X');
        g_flash_test = TestFlash();
        if(g_flash_test != 0x0E) led.ColorOrder('R');
        else led.ColorOrder('B');
    }
}

uint8_t Run::TestFlash()
{
    uint32_t address_1 = SECTOR_1_ADDRESS_HEAD;
    uint32_t address_2 = SECTOR_2_ADDRESS_HEAD;
    uint32_t address_3 = SECTOR_3_ADDRESS_HEAD;
    uint32_t address_4 = SECTOR_4_ADDRESS_HEAD;
    uint32_t i, num_of_data = 3;
    uint8_t a[num_of_data] = {1,2,3};
    uint16_t b[num_of_data] = {4,5,6};
    int16_t c[num_of_data] = {-7,-8,-9};
    float d[num_of_data] = {0.1,0.2,0.3};
    uint8_t buff_a[num_of_data] = {0};
    uint16_t buff_b[num_of_data] = {0};
    int16_t buff_c[num_of_data] = {0};
    float buff_d[num_of_data] = {0};

    if(!flash.Clear()) return 0x01;

    if(!flash.CheckBlankByte(address_1, num_of_data)) return 0x02;
    if(!flash.StoreUint8(address_1, a, num_of_data)) return 0x03;
    flash.Load(buff_a, address_1, num_of_data);
    for(i = 0; i < num_of_data; i++) if(a[i] != buff_a[i]) return 0x04;

    if(!flash.CheckBlankHalfword(address_2, num_of_data)) return 0x05;
    if(!flash.StoreUint16(address_2, b, num_of_data)) return 0x06;
    flash.Load(buff_b, address_2, num_of_data*2);
    for(i = 0; i < num_of_data; i++) if(b[i] != buff_b[i]) return 0x07;

    if(!flash.CheckBlankHalfword(address_3, num_of_data)) return 0x08;
    if(!flash.StoreInt16(address_3, c, num_of_data)) return 0x09;
    flash.Load(buff_c, address_3, num_of_data*2);
    for(i = 0; i < num_of_data; i++) if(c[i] != buff_c[i]) return 0x0A;

    if(!flash.CheckBlankWord(address_4, num_of_data)) return 0x0B;
    if(!flash.StoreFloat(address_4, d, num_of_data)) return 0x0C;
    flash.Load(buff_d, address_4, num_of_data*4);
    for(i = 0; i < num_of_data; i++) if(d[i] != buff_d[i]) return 0x0D;

    return 0x0E;
}
#endif // DEBUG_FLASH

#ifdef DEBUG_MODE
void Run::InitialTest()
{
    /* Line sensor */
    line_sensor.Update();
    line_sensor.MonitorArrays();
    g_line_diff = line_sensor.LeftRightDifference();
    g_line_emer = line_sensor.GetEmergencyStopFlag();
    g_line_calib = line_sensor.CheckCalibration();

    /* Side seneor */
    side_sensor.IgnoreJudgment();
    g_goal_cnt = side_sensor.GetGoalMarkerCount();
    g_corner_cnt = side_sensor.GetCornerMarkerCount();
    g_cross_cnt = side_sensor.GetCrossLineCount();

    /* IMU */
    iim_42652.Update();
    g_deg_stack_z = iim_42652.GetDegreeStackZ();
    g_gyro_x_l = iim_42652.GyroXLeft();
    g_gyro_x_r = iim_42652.GyroXRight();
    g_gyro_y_l = iim_42652.GyroYLeft();
    g_gyro_y_r = iim_42652.GyroYRight();
    g_gyro_z_l = iim_42652.GyroZLeft();
    g_gyro_z_r = iim_42652.GyroZRight();
    g_accel_x_l = iim_42652.AccelXLeft();
    g_accel_x_r = iim_42652.AccelXRight();
    g_accel_y_l = iim_42652.AccelYLeft();
    g_accel_y_r = iim_42652.AccelYRight();
    g_accel_z_l = iim_42652.AccelZLeft();
    g_accel_z_r = iim_42652.AccelZRight();

    /* Encoder */
    encoder.Update();
    g_distance = encoder.GetDistance();
    g_distance_stack = encoder.GetDistanceStack();
    g_distance_diff = encoder.AngularVelocity();

    /* Motor */
    motor.Drive(INIT_DEBUG_MOTOR_DUTY, 0);
}
#endif // DEBUG_MODE