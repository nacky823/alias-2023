#include "run.hpp"
#include "declare_extern.h"

Run::Run(Encoder *encoder,
         Flash *flash,
         Iim42652 *iim_42652,
         Imu *imu,
         Led *led,
         LineSensor *line_sensor,
         LineTrace *line_trace,
         Logger *logger,
         Motor *motor,
         Print *print,
         RotarySwitch *rotary_switch,
         SideSensor *side_sensor,
         VelocityControl *velocity_control
         ) : run_mode_(0)
           , run_mode_changed_internal_(false)
           , switch_interval_wait_timer_(0)
           , switch_interval_wait_enable_(true)
           , switch_interval_led_enable_(true)
           , mode_complete_(true)
           , store_log_failed_(true)
{
    encoder_          = encoder;
    flash_            = flash;
    iim_42652_        = iim_42652;
    imu_              = imu;
    led_              = led;
    line_sensor_      = line_sensor;
    line_trace_       = line_trace;
    logger_           = logger;
    motor_            = motor;
    print_            = print;
    rotary_switch_    = rotary_switch;
    side_sensor_      = side_sensor;
    velocity_control_ = velocity_control;
}

void Run::Init()
{
    line_sensor_->Init();
    encoder_->Init();
    motor_->Init();
    uint8_t imu_init = iim_42652_->Init();

    bool flash_erase = true;
    uint8_t switch_state = rotary_switch_->State();

    if(switch_state == 0x0F)
    {
        led_->Blink(3, 'R', 'X');
        if(!flash_->Clear()) flash_erase = false;
    }
    else if(switch_state == 0x0B)
    {
        led_->Blink(3, 'M', 'X');
        print_->Log();
        return;
    }

#ifdef DEBUG_MODE
    g_imu_init = imu_init;
    g_flash_erase = flash_erase;
    g_switch_state = switch_state;
#endif // DEBUG_MODE

    if(flash_erase && imu_init == 0x09)
    {
        led_->Rainbow(1);
        HAL_TIM_Base_Start_IT(&htim2);
        HAL_TIM_Base_Start_IT(&htim6);
        HAL_TIM_Base_Start_IT(&htim7);
    }
    else if(flash_erase) led_->ColorOrder('Y');
    else led_->ColorOrder('R');
}

void Run::Timer7()
{
    line_sensor_->StoreConsecutiveBuff();
}

void Run::Timer2()
{
    uint8_t switch_state = rotary_switch_->State();
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
        led_->ResetInterrupt();
        interval_enable = true;
    }
    else if(interval_enable)
    {
        SetRunMode(STANDBY);
        interval_enable = SwitchChangeInterval(switch_state);
    }
    else UpdateRunMode(switch_state);

    pre_switch_state = switch_state;
}

void Run::UpdateRunMode(uint8_t switch_state)
{
    if(run_mode_changed_internal_) return;

    switch(switch_state)
    {
#ifdef DEBUG_MODE
        case 0x03: SetRunMode(GYRO_TEST); break;
        case 0x02: SetRunMode(LINE_TRACE_DEBUG); break;
        case 0x01: SetRunMode(VELOCITY_CONTROL_DEBUG); break;
#else // DEBUG_MODE
        case 0x02: SetRunMode(LINE_TRACE); break;
        case 0x01: SetRunMode(VELOCITY_CONTROL); break;
#endif // DEBUG_MODE

        case 0x0E: SetRunMode(READY); break;
        case 0x0C: SetRunMode(DEV); break;
        case 0x0A: SetRunMode(DEV_ACCEL); break;
        case 0x08: SetRunMode(THIRD_RUN); break;
        case 0x06: SetRunMode(FORTH_RUN); break;
        case 0x04: SetRunMode(FIFTH_RUN); break;
        default:   SetRunMode(STANDBY); break;
    }
}

void Run::SetRunMode(uint8_t mode)
{
    run_mode_ = mode;
}

void Run::SetRunModeChangedInternal(bool boolean)
{
    run_mode_changed_internal_ = boolean;

#ifdef DEBUG_MODE
    g_run_mode_changed_internal_cnt++;
#endif // DEBUG_MODE
}

void Run::WhenSwitchChange()
{
    SetRunModeChangedInternal(false);
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
            case 0x0A: led_enable = led_->BlinkInterrupt(3, 'G', 'B'); break;
            case 0x0B: led_enable = led_->BlinkInterrupt(3, 'G', 'Y'); break;
#endif // DEBUG_MODE

            case 0x0C: led_enable = led_->BlinkInterrupt(3, 'B', 'M'); break;
            case 0x0D: led_enable = led_->BlinkInterrupt(3, 'G', 'X'); break;
            case 0x0F: led_enable = led_->BlinkInterrupt(3, 'Y', 'X'); break;
            case 0x01: led_enable = led_->BlinkInterrupt(3, 'B', 'X'); break;
            case 0x02: led_enable = led_->BlinkInterrupt(3, 'M', 'X'); break;
            case 0x07: led_enable = led_->BlinkInterrupt(3, 'G', 'B'); break;
            case 0x08: led_enable = led_->BlinkInterrupt(3, 'G', 'Y'); break;
            case 0x03: led_enable = led_->BlinkInterrupt(3, 'G', 'B'); break;
            default:   led_enable = led_->BlinkInterrupt(3, 'X', 'W'); break;
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
    bool line_emergency = line_sensor_->GetEmergencyStopFlag();

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
        case LINE_TRACE_DEBUG: ModeLineTraceDebug(); break;
        case VELOCITY_CONTROL_DEBUG: ModeVelocityControlDebug(); break;
#else // DEBUG_MODE
        case LINE_TRACE: ModeLineTrace(); break;
        case VELOCITY_CONTROL: ModeVelocityControl(); break;
#endif // DEBUG_MODE

        case EMERGENCY: ModeEmergency(); break;
        case STANDBY: ModeStandby(); break;
        case READY: ModeReady(); break;
        case DEV: ModeDevelopment(); break;
        case DEV_ACCEL: ModeDevAccel(); break;
        case DEV_GOAL: ModeDevGoal(); break;
        case FIRST_RUN: ModeFirstRun(); break;
        case FIRST_GOAL: ModeFirstGoal(); break;
        case SECOND_RUN: ModeSecondRun(); break;
        case SECOND_GOAL: ModeSecondGoal(); break;
        case GYRO_TEST: ModeGyroTest(); break;
        default: ModeStandby(); break;
    }

    mode_complete_ = true;
}

void Run::ModeEmergency()
{
    motor_->Drive(0, 0);

    bool result = logger_->GetSuccessEmergencyCodeStore();
    if(result) led_->ColorOrder('R');
    else led_->ColorOrder('G');
}

void Run::ModeReady()
{
    line_sensor_->Update();
    if(line_sensor_->CheckCalibration()) led_->ColorOrder('X');
    else led_->ColorOrder('R');
}

void Run::ModeStandby()
{
    motor_->Drive(0, 0);
}

/*
void Run::ModeDevelopment()
{
    encoder_->Update();
    line_sensor_->Update();
    side_sensor_->Update();

    if(DevEmergencyStop()) return;

    uint8_t goal_count = side_sensor_->GetGoalMarkerCount();
    float target_velocity = DevTargetVelocity(goal_count);
    float trans_ratio = velocity_control_->DeterminePidGain(target_velocity);
    float rotat_ratio = line_trace_->DeterminePidGain(target_velocity);
    motor_->Drive(trans_ratio, rotat_ratio);
}
*/

void Run::ModeGyroTest()
{
    imu_->Update();
    float imu_rad_z = imu_->GetRadZ();
    float imu_ratio = imu_->PidControl(imu_rad_z);
    g_run_pid_ratio = imu_ratio;
    motor_->Drive(0, imu_ratio);
}

void Run::ModeDevelopment()
{
    /* Sensor update */
    encoder_->Update();
    line_sensor_->Update();
    side_sensor_->Update();
    imu_->Update();
    logger_->Logging();
    /* Emergency stop */
    if(DevEmergencyStop()) return;
    /* Motor control */
    uint8_t goal_count = side_sensor_->GetGoalMarkerCount();
    float target_velocity = DevTargetVelocity(goal_count);
    float trans_ratio = velocity_control_->DeterminePidGain(target_velocity);
    float rotat_ratio = line_trace_->DeterminePidGain(target_velocity);
    motor_->Drive(trans_ratio, rotat_ratio);
    monitor_count++;
}

void Run::ModeDevAccel()
{
    /* Sensor update */
    encoder_->Update();
    line_sensor_->Update();
    side_sensor_->Update();
    logger_->Loading();
    /* Motor control */
    uint8_t goal_count = side_sensor_->GetGoalMarkerCount();
    float target_velocity = DevAccelTarget(goal_count);
    float trans_ratio = velocity_control_->DeterminePidGain(target_velocity);
    float rotat_ratio = line_trace_->DeterminePidGain(target_velocity);
    /* Emergency stop */
    if(DevEmergencyStop()) return;
    motor_->Drive(trans_ratio, rotat_ratio);
}

bool Run::DevEmergencyStop()
{
    static uint8_t emergency_timer = 0;
    bool line_emergency = line_sensor_->GetEmergencyStopFlag();
    bool logging_emergency = logger_->GetEmergencyStopFlag();

    if(logging_emergency)
    {
        SetRunMode(EMERGENCY);
        SetRunModeChangedInternal(true);
        return true;
    }

    if(line_emergency)
    {
        if(emergency_timer >= EMERGENCY_STOP_TIME)
        {
            SetRunMode(EMERGENCY);
            SetRunModeChangedInternal(true);
            return true;
        }
        else emergency_timer++;
    }
    else emergency_timer = 0;

    return false;
}

float Run::DevTargetVelocity(uint8_t goal_count)
{
    static uint16_t slow_timer = 0;
    static uint16_t stop_timer = 0;
    float target_velocity;

    switch(goal_count)
    {
        case 0:
        case 1: target_velocity = EXPLORE_VELOCITY; break;
        default:

            if(slow_timer < SLOW_DRIVE_TIME)
            {
                slow_timer++;
                target_velocity = SLOW_DRIVE_VELOCITY;
            }
            else
            {
                if(stop_timer < STOP_TIME)
                {
                    stop_timer++;
                    target_velocity = 0;
                }
                else
                {
                    SetRunMode(DEV_GOAL);
                    SetRunModeChangedInternal(true);
                    target_velocity = 0;
                }
            }
            break;
    }

    return target_velocity;
}

float Run::DevAccelTarget(uint8_t goal_count)
{
    static uint16_t slow_timer = 0;
    static uint16_t stop_timer = 0;
    float target_velocity;

    switch(goal_count)
    {
        case 0: target_velocity = MIN_VELOCITY; break;
        case 1:
            target_velocity = logger_->GetTargetVelocity();
            break;
        default:

            if(slow_timer < SLOW_DRIVE_TIME)
            {
                slow_timer++;
                target_velocity = SLOW_DRIVE_VELOCITY;
            }
            else
            {
                if(stop_timer < STOP_TIME)
                {
                    stop_timer++;
                    target_velocity = 0;
                }
                else
                {
                    SetRunMode(DEV_GOAL);
                    SetRunModeChangedInternal(true);
                    target_velocity = 0;
                }
            }
            break;
    }

    return target_velocity;
}

void Run::ModeDevGoal()
{
    motor_->Drive(0, 0);
    led_->ColorOrder('M');
}

void Run::ModeFirstRun()
{
    /* Sensor update */
    line_sensor_->Update();
    encoder_->Update();
    imu_->Update();
    side_sensor_->Update();
    /* Emergency stop */
    if(EmergencyStop()) return;
    /* Logging */
    uint8_t goal_count = side_sensor_->GetGoalMarkerCount();
    //if(goal_count == 1) logger_->Logging(mode_complete_);
    //mode_complete_ = false;
    //uint8_t period_success = logger_->StorePeriodicLog();
    //uint8_t accel_success = logger_->StoreAccelPositionLog();
    //if(period_success != 0 || !accel_success != 0) store_log_failed_ = true;
    /* Motor control */
    float target_velocity = FirstTargetVelocity(goal_count);
    float trans_ratio = velocity_control_->DeterminePidGain(target_velocity);
    float rotat_ratio = line_trace_->DeterminePidGain(target_velocity);
    motor_->Drive(trans_ratio, rotat_ratio);
}

void Run::ModeFirstGoal()
{
    motor_->Drive(0, 0);
    if(store_log_failed_) led_->ColorOrder('R');
    else led_->ColorOrder('B');
}

void Run::ModeSecondRun()
{
    /* Sensor update */
    line_sensor_->Update();
    encoder_->Update();
    imu_->Update();
    side_sensor_->Update();
    /* Emergency stop */
    if(EmergencyStop()) return;
    /* Loading */
    uint8_t goal_count = side_sensor_->GetGoalMarkerCount();
    if(goal_count == 1) logger_->Loading();
    /* Motor control */
    float target_velocity = SecondTargetVelocity(goal_count);
    float trans_ratio = velocity_control_->DeterminePidGain(target_velocity);
    float rotat_ratio = line_trace_->DeterminePidGain(target_velocity);
    motor_->Drive(trans_ratio, rotat_ratio);
}

void Run::ModeSecondGoal()
{
    motor_->Drive(0, 0);
    led_->ColorOrder('M');
}

void Run::ModeVelocityControl()
{
    /* Sensor update */
    encoder_->Update();
    line_sensor_->Update();
    side_sensor_->Update();
    /* Emergency stop */
    if(EmergencyStop()) return;
    /* Motor control */
    uint8_t goal_count = side_sensor_->GetGoalMarkerCount();
    float target_velocity = VelocityControlTarget(goal_count);
    float trans_ratio = velocity_control_->DeterminePidGain(target_velocity);
    float rotat_ratio = line_trace_->DeterminePidGain(target_velocity);
    motor_->Drive(trans_ratio, rotat_ratio);
}

void Run::ModeLineTrace()
{
    /* Sensor update */
    line_sensor_->Update();
    side_sensor_->Update();
    /* Emergency stop */
    if(EmergencyStop()) return;
    /* Motor control */
    uint8_t goal_count = side_sensor_->GetGoalMarkerCount();
    float trans_ratio = CommonDuty(goal_count);
    float rotat_ratio = line_trace_->LineTraceOnly();
    motor_->Drive(trans_ratio, rotat_ratio);
}

#ifdef DEBUG_MODE
void Run::ModeVelocityControlDebug()
{
    /* Sensor update */
    line_sensor_->Update();
    encoder_->Update();
    /* Motor control */
    float target_velocity = VELOCITY_CONTROL_TARGET;
    float trans_ratio = velocity_control_->DeterminePidGain(target_velocity);
    motor_->Drive(trans_ratio, 0);
}

void Run::ModeLineTraceDebug()
{
    /* Sensor update */
    line_sensor_->Update();
    /* Motor control */
    float rotat_ratio = line_trace_->LineTraceOnly();
    motor_->Drive(0, rotat_ratio);
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
        return logger_->GetTargetVelocity();
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
    if(rotary_switch_->State() == 0x0A)
    {
        led_->Blink(5, 'R', 'X');
        g_flash_test = TestFlash();
        if(g_flash_test != 0x0E) led_->ColorOrder('R');
        else led_->ColorOrder('B');
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

    if(!flash_->Clear()) return 0x01;

    if(!flash_->CheckBlankByte(address_1, num_of_data)) return 0x02;
    if(!flash_->StoreUint8(address_1, a, num_of_data)) return 0x03;
    flash_->Load(buff_a, address_1, num_of_data);
    for(i = 0; i < num_of_data; i++) if(a[i] != buff_a[i]) return 0x04;

    if(!flash_->CheckBlankHalfword(address_2, num_of_data)) return 0x05;
    if(!flash_->StoreUint16(address_2, b, num_of_data)) return 0x06;
    flash_->Load(buff_b, address_2, num_of_data*2);
    for(i = 0; i < num_of_data; i++) if(b[i] != buff_b[i]) return 0x07;

    if(!flash_->CheckBlankHalfword(address_3, num_of_data)) return 0x08;
    if(!flash_->StoreInt16(address_3, c, num_of_data)) return 0x09;
    flash_->Load(buff_c, address_3, num_of_data*2);
    for(i = 0; i < num_of_data; i++) if(c[i] != buff_c[i]) return 0x0A;

    if(!flash_->CheckBlankWord(address_4, num_of_data)) return 0x0B;
    if(!flash_->StoreFloat(address_4, d, num_of_data)) return 0x0C;
    flash_->Load(buff_d, address_4, num_of_data*4);
    for(i = 0; i < num_of_data; i++) if(d[i] != buff_d[i]) return 0x0D;

    return 0x0E;
}
#endif // DEBUG_FLASH

#ifdef DEBUG_MODE
void Run::InitialTest()
{
    /* Line sensor */
    line_sensor_->Update();
    line_sensor_->MonitorLineBuff();
    g_line_diff = line_sensor_->LeftRightDifference();
    g_line_emer = line_sensor_->GetEmergencyStopFlag();
    g_line_calib = line_sensor_->CheckCalibration();

    /* Side seneor */
    side_sensor_->Update();
    g_goal_cnt = side_sensor_->GetGoalMarkerCount();
    g_corner_cnt = side_sensor_->GetCornerMarkerCount();
    g_cross_cnt = side_sensor_->GetCrossLineCount();

    /* IMU */
    imu_->Update();
    g_gyro_x_l = iim_42652_->GyroXLeft();
    g_gyro_x_r = iim_42652_->GyroXRight();
    g_gyro_y_l = iim_42652_->GyroYLeft();
    g_gyro_y_r = iim_42652_->GyroYRight();
    g_gyro_z_l = iim_42652_->GyroZLeft();
    g_gyro_z_r = iim_42652_->GyroZRight();
    g_accel_x_l = iim_42652_->AccelXLeft();
    g_accel_x_r = iim_42652_->AccelXRight();
    g_accel_y_l = iim_42652_->AccelYLeft();
    g_accel_y_r = iim_42652_->AccelYRight();
    g_accel_z_l = iim_42652_->AccelZLeft();
    g_accel_z_r = iim_42652_->AccelZRight();

    /* Encoder */
    encoder_->Update();
    g_distance = encoder_->GetDistance();
    g_distance_stack = encoder_->GetDistanceStack();
    g_distance_diff = encoder_->AngularVelocity();

    /* Motor */
    motor_->Drive(INIT_DEBUG_MOTOR_DUTY, 0);
}
#endif // DEBUG_MODE