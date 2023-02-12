#include "wrapper.hpp"
#include "encoder.hpp"
#include "flash.hpp"
#include "iim_42652.hpp"
#include "led.hpp"
#include "line_sensor.hpp"
#include "line_trace.hpp"
#include "logger.hpp"
#include "motor.hpp"
#include "rotary_switch.hpp"
#include "side_sensor.hpp"
#include "velocity_control.hpp"
#include "macro.h"
#include "declare_extern.h"
#include <string.h>

#define SWITCH_CHANGE_INTERVAL_MS 3000 // [ms]
#define EMERGENCY_STOP_COUNT 5 // tim6 [ms]
#define GOAL_SLOW_COUNT 10     // tim6 [ms]
#define GOAL_SLOW_VELOCITY 1.0 // [mm/ms]
#define GOAL_STOP_COUNT 2000   // tim6 [ms]
/* g_mode */
#define VELOCITY_CONTROL_DEBUG 0x0C
#define LINE_TRACE_DEBUG 0x0D
#define INITIAL_DEBUG 0x0E
#define READY 0x0F
#define STANDBY 0x00
#define FIRST_RUN 0x01
#define FIRST_GOAL 0x10
#define SECOND_RUN 0x02
#define SECOND_GOAL 0x20
#define VELOCITY_CONTROL 0x04
#define LINE_TRACE 0x05

uint8_t FlashTest();

Encoder encoder;
Flash flash;
Iim42652 iim_42652;
Led led;
LineSensor line_sensor;
LineTrace line_trace;
Logger logger;
Motor motor;
RotarySwitch rotary_switch;
SideSensor side_sensor;
VelocityControl velocity_control;

void Init()
{
    line_sensor.Init();
    encoder.Init();
    motor.Init();
    uint8_t imu_init = iim_42652.Init();

    HAL_TIM_Base_Start_IT(&htim2);
    HAL_TIM_Base_Start_IT(&htim5);
    HAL_TIM_Base_Start_IT(&htim6);
    HAL_TIM_Base_Start_IT(&htim7);

    if(imu_init == 0x09) led.Rainbow(2);

#ifdef DEBUG_MODE
    g_imu_init = imu_init;
#endif // DEBUG_MODE
}


void ExternalInterrupt(uint16_t gpio_pin)
{
    switch(gpio_pin)
    {
        case GPIO_PIN_12:
        case GPIO_PIN_2:
        case GPIO_PIN_1:
        case GPIO_PIN_0:
        default: g_main_while_reset = 1; led.ColorOrder('X'); break;
    }

#ifdef DEBUG_MODE
    g_external_interrupt++;
#endif // DEBUG_MODE
}


void InterruptTim7()
{
    line_sensor.StoreConsecutiveAdcBuffers();

#ifdef DEBUG_MODE
    g_tim7++;
#endif // DEBUG_MODE
}


void InterruptTim6()
{
    if(g_tim6_complete == 0) g_tim6_yet = 0x01;
    g_tim6_complete = 0;

    switch(g_mode)
    {
        case READY:
            line_sensor.UpdateAdcValues();
            g_line_calib = line_sensor.CheckCalibration();
            if(g_line_calib == 0) led.ColorOrder('X');
            else led.ColorOrder('R');
            break;

        case STANDBY:
            motor.Drive(0, 0);
            break;

        case FIRST_RUN:
            /* Sensor update */
            line_sensor.UpdateAdcValues();
            encoder.Update();
            side_sensor.IgnoreJudgment();
            g_goal_cnt = side_sensor.GetGoalMarkerCount();
            /* Motor control */
            g_target = TargetVelocity(MIN_VELOCITY, MIN_VELOCITY);
            g_trans = velocity_control.PidControl(g_target, V_P_1, V_I_1, V_D_1);
            g_rotat = line_trace.PidControl(LINE_P_1, LINE_I_1, LINE_D_1);
            motor.Drive(g_trans, g_rotat);
            EmergencyStop();
            break;
        
        case FIRST_GOAL:
            motor.Drive(0, 0);
            if(g_first_log_failed == 1) led.ColorOrder('R');
            else led.ColorOrder('B');
            break;

        case SECOND_RUN:
            /* Sensor update */
            line_sensor.UpdateAdcValues();
            encoder.Update();
            side_sensor.IgnoreJudgment();
            g_goal_cnt = side_sensor.GetGoalMarkerCount();
            /* Motor control */
            g_target = TargetVelocity(logger.GetTargetVelocity(), MIN_VELOCITY);
            g_trans = velocity_control.PidControl(g_target, V_P_1, V_I_1, V_D_1);
            g_rotat = line_trace.PidControl(LINE_P_1, LINE_I_1, LINE_D_1);
            motor.Drive(g_trans, g_rotat);
            EmergencyStop();
            if(g_run_end == 1) g_mode = SECOND_GOAL;
            break;

        case SECOND_GOAL:
            motor.Drive(0, 0);
            break;

        case LINE_TRACE:
            /* Sensor update */
            line_sensor.UpdateAdcValues();
            encoder.Update();
            side_sensor.IgnoreJudgment();
            g_goal_cnt = side_sensor.GetGoalMarkerCount();
            /* Motor control */
            g_common_speed = TargetDuty(COMMON_DUTY, COMMON_DUTY);
            g_rotat = line_trace.PidControl(LINE_P_1, LINE_I_1, LINE_D_1);
            motor.Drive(g_common_speed, g_rotat);
            EmergencyStop();
            if(g_run_end == 1)
            {
                led.ColorOrder('Y');
                g_mode = STANDBY;
            }
            break;

        case VELOCITY_CONTROL:
            /* Sensor update */
            line_sensor.UpdateAdcValues();
            encoder.Update();
            side_sensor.IgnoreJudgment();
            g_goal_cnt = side_sensor.GetGoalMarkerCount();
            /* Motor control */
            g_target = TargetVelocity(MIN_VELOCITY, MIN_VELOCITY);
            g_trans = velocity_control.PidControl(g_target, V_P_1, V_I_1, V_D_1);
            g_rotat = line_trace.PidControl(LINE_P_1, LINE_I_1, LINE_D_1);
            motor.Drive(g_trans, g_rotat);
            EmergencyStop();
            if(g_run_end == 1)
            {
                led.ColorOrder('C');
                g_mode = STANDBY;
            }
            break;

        default:
            motor.Drive(0, 0); break;
    }
    g_tim6_complete = 1;

#ifdef DEBUG_MODE
    g_tim6++;
#endif // DEBUG_MODE
}


void InterruptTim5()
{
    if(g_tim5_complete == 0) g_tim5_yet = 0x02;
    g_tim5_complete = 0;

    switch(g_mode)
    {
        case FIRST_RUN:
            iim_42652.Update();
            g_process_complete = 0;
            g_process_complete |= g_tim6_yet;
            g_process_complete |= g_tim5_yet;
            g_process_complete |= g_tim2_yet;
            if(g_goal_cnt == 1) logger.Logging(g_process_complete);
            g_tim6_yet = 0;
            g_tim5_yet = 0;
            g_tim2_yet = 0;
            break;

        case SECOND_RUN:
            iim_42652.Update();
            if(g_goal_cnt == 1) logger.Loading();
            break;

        default: break;
    }
    g_tim5_complete = 1;

#ifdef DEBUG_MODE
    g_tim5++;
#endif // DEBUG_MODE
}


void InterruptTim2()
{
    if(g_tim2_complete == 0) g_tim2_yet = 0x02;
    g_tim2_complete = 0;

    if(g_emergency_stop == 1) g_mode =STANDBY;

    switch(g_mode)
    {
        case FIRST_RUN:
            g_store_periodic_log = logger.StorePeriodicLog();
            g_store_accel_log = logger.StoreAccelPositionLog();
            if(g_store_periodic_log + g_store_accel_log != 0) g_first_log_failed = 1;

            if(g_run_end == 1) g_mode = FIRST_GOAL;
            break;

        default: break;
    }
    g_tim2_complete = 1;

#ifdef DEBUG_MODE
    g_tim2++;
#endif // DEBUG_MODE
}


void Loop()
{
    g_main_while_reset = 0;
    g_switch_state = rotary_switch.State();

    switch(g_switch_state)
    {
#ifdef DEBUG_MODE
        case 0x0B: // Flash debug
            HAL_Delay(SWITCH_CHANGE_INTERVAL_MS);
            if(g_main_while_reset == 1) break;
            led.Blink(3, 'R', 'X');
            g_mode = STANDBY;
            HAL_Delay(SWITCH_CHANGE_INTERVAL_MS);
            if(g_main_while_reset == 1) break;

            led.Blink(3, 'R', 'X');
            g_flash_test = FlashTest();

            while(g_main_while_reset == 0) {}
            break;

        case 0x0C: // Velocity control debug
            HAL_Delay(SWITCH_CHANGE_INTERVAL_MS);
            if(g_main_while_reset == 1) break;

            led.Blink(3, 'G', 'B');
            led.ColorOrder('X');
            g_mode = VELOCITY_CONTROL_DEBUG;

            while(g_main_while_reset == 0) {}
            break;

        case 0x0D: // Line trace debug
            HAL_Delay(SWITCH_CHANGE_INTERVAL_MS);
            if(g_main_while_reset == 1) break;

            led.Blink(3, 'G', 'Y');
            led.ColorOrder('X');
            g_mode = LINE_TRACE_DEBUG;

            while(g_main_while_reset == 0) {}
            break;

        case 0x0E: // Initial debug
            HAL_Delay(SWITCH_CHANGE_INTERVAL_MS);
            if(g_main_while_reset == 1) break;

            led.Blink(3, 'G', 'X');
            g_mode = INITIAL_DEBUG;

            while(g_main_while_reset == 0) {}
            break;
#else // DEBUG_MODE
        
        case 0x0E: // Flash erase
            HAL_Delay(SWITCH_CHANGE_INTERVAL_MS);
            if(g_main_while_reset == 1) break;
            led.Blink(3, 'R', 'X');
            g_mode = STANDBY;
            HAL_Delay(SWITCH_CHANGE_INTERVAL_MS);
            if(g_main_while_reset == 1) break;

            led.Blink(3, 'R', 'X');
            g_flash_erase = 0;
            if(!flash.Clear()) flash_erase = 1;

            while(g_main_while_reset == 0)
            {
                if(flash_erase == 0) led.Rainbow(1);
            }
            break;
#endif // DEBUG_MODE

        case 0x0F:
            HAL_Delay(SWITCH_CHANGE_INTERVAL_MS);
            if(g_main_while_reset == 1) break;

            led.Blink(3, 'Y', 'X');
            g_mode = READY;

            while(g_main_while_reset == 0) {}
            break;

        case 0x00:
            HAL_Delay(SWITCH_CHANGE_INTERVAL_MS);
            if(g_main_while_reset == 1) break;

            g_mode = STANDBY;

            while(g_main_while_reset == 0) { led.Rainbow(1); }
            break;

        case 0x01:
            HAL_Delay(SWITCH_CHANGE_INTERVAL_MS);
            if(g_main_while_reset == 1) break;

            led.Blink(3, 'B', 'X');
            g_mode = FIRST_RUN;

            while(g_main_while_reset == 0) {}
            break;

        case 0x02:
            HAL_Delay(SWITCH_CHANGE_INTERVAL_MS);
            if(g_main_while_reset == 1) break;

            led.Blink(3, 'M', 'B');
            led.ColorOrder('X');
            g_mode = SECOND_RUN;

            while(g_main_while_reset == 0) {}
            break;

        case 0x04: // Velocity control
            HAL_Delay(SWITCH_CHANGE_INTERVAL_MS);
            if(g_main_while_reset == 1) break;

            led.Blink(3, 'G', 'B');
            led.ColorOrder('X');
            g_mode = VELOCITY_CONTROL;

            while(g_main_while_reset == 0) {}
            break;

        case 0x05: // Line trace
            HAL_Delay(SWITCH_CHANGE_INTERVAL_MS);
            if(g_main_while_reset == 1) break;

            led.Blink(3, 'G', 'Y');
            led.ColorOrder('X');
            g_mode = LINE_TRACE;

            while(g_main_while_reset == 0) {}
            break;

        default:
            HAL_Delay(SWITCH_CHANGE_INTERVAL_MS);
            if(g_main_while_reset == 1) break;

            g_mode = STANDBY;

            while(g_main_while_reset == 0) { led.Blink(1, 'W', 'X'); }
            break;
    }
}


void EmergencyStop()
{
    static uint8_t emergency_cnt = 0;

    if(emergency_cnt >= EMERGENCY_STOP_COUNT)
    {
        motor.Drive(0, 0);
        led.ColorOrder('W');
        g_emergency_stop = 1;
    }
    else if(line_sensor.GetEmergencyStopFlag() == 1) emergency_cnt++;
    else emergency_cnt = 0;
}


float TargetVelocity(float target, float min)
{
    static uint8_t slow_cnt = 0;
    static uint8_t stop_cnt = 0;

    if(g_goal_cnt == 1) return target;
    else if(g_goal_cnt >= 2)
    {
        if(slow_cnt < GOAL_SLOW_COUNT)
        {
            slow_cnt++; return GOAL_SLOW_VELOCITY;
        }
        else if(stop_cnt < GOAL_STOP_COUNT)
        {
            stop_cnt++; return 0;
        }
        else
        {
            g_run_end = 1; return 0;
        }
    }
    else return min;
}

float TargetDuty(float target, float min)
{
    static uint8_t slow_cnt = 0;
    static uint8_t stop_cnt = 0;

    if(g_goal_cnt == 1) return target;
    else if(g_goal_cnt >= 2)
    {
        if(slow_cnt < GOAL_SLOW_COUNT)
        {
            slow_cnt++; return min * 0.6;
        }
        else if(stop_cnt < GOAL_STOP_COUNT)
        {
            stop_cnt++; return 0;
        }
        else
        {
            g_run_end = 1; return 0;
        }
    }
    else return min;
}

#ifdef DEBUG_MODE

uint8_t FlashTest()
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
    memcpy(buff_a, reinterpret_cast<uint32_t*>(address_1), num_of_data);
    for(i = 0; i < num_of_data; i++) if(a[i] != buff_a[i]) return 0x05;

    if(!flash.CheckBlankHalfword(address_2, num_of_data)) return 0x06;
    if(!flash.StoreUint16(address_2, b, num_of_data)) return 0x07;
    //if(!flash.Load(buff_b, address_2, num_of_data*2)) return 0x08;
    for(i = 0; i < num_of_data; i++) if(b[i] != buff_b[i]) return 0x09;

    if(!flash.CheckBlankHalfword(address_3, num_of_data)) return 0x0A;
    if(!flash.StoreInt16(address_3, c, num_of_data)) return 0x0B;
    //if(!flash.Load(buff_c, address_3, num_of_data*2)) return 0x0C;
    for(i = 0; i < num_of_data; i++) if(c[i] != buff_c[i]) return 0x0D;

    if(!flash.CheckBlankWord(address_4, num_of_data)) return 0x0E;
    if(!flash.StoreFloat(address_4, d, num_of_data)) return 0x0F;
    //if(!flash.Load(buff_d, address_4, num_of_data*4)) return 0x10;
    for(i = 0; i < num_of_data; i++) if(d[i] != buff_d[i]) return 0x11;

    return 0x12;
}

void InitialTest()
{
    /* Line sensor */
    line_sensor.UpdateAdcValues();
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

uint8_t ImuReadByte(uint8_t send_address, char imu_ic_lr)
{
    return iim_42652.Read(send_address, imu_ic_lr);
}

void ImuWriteByte(uint8_t send_address, uint8_t send_data, char imu_ic_lr)
{
    iim_42652.Write(send_address, send_data, imu_ic_lr);
}
#endif // DEBUG_MODE