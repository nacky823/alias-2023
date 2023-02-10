#include "wrapper.hpp"
#include "encoder.hpp"
#include "iim_42652.hpp"
#include "led.hpp"
#include "line_sensor.hpp"
#include "line_trace.hpp"
#include "motor.hpp"
#include "rotary_switch.hpp"
#include "side_sensor.hpp"
#include "declare_extern.h"
#include "velocity_control.hpp"

Encoder encoder;
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
    if(gpio_pin == GPIO_PIN_12)     g_main_while_reset = 1;
    else if(gpio_pin == GPIO_PIN_2) g_main_while_reset = 1;
    else if(gpio_pin == GPIO_PIN_1) g_main_while_reset = 1;
    else if(gpio_pin == GPIO_PIN_0) g_main_while_reset = 1;

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
    switch(g_mode)
    {
        case RESET:
            motor.Drive(0, 0);
            led.ColorOrder('G');
            break;

        case READY:
            line_sensor.UpdateAdcValues();
            g_line_calib = line_sensor.CheckCalibration();
            if(g_line_calib == 1) led.ColorOrder('R');
            else if(g_line_calib == 0) led.ColorOrder('X');
            else led.ColorOrder('W');
            break;

        case STANDBY:
            motor.Drive(0, 0);
            break;

        case FIRST_RUN:
            encoder.UpdateCountDistance();
            line_sensor.UpdateAdcValues();
            float rotat = line_sensor.PidControl(LINE_KP_1, LINE_KI_1, LINE_KD_1);
            float trans = velocity_control.PidControl(TARGET_V_1, V_KP_1, V_KI_1, V_KD_1);
            motor.Drive(trans, rotat);
            EmergencyStop();
            side_sensor.IgnoreJudgment();
            if(side_sensor.GetGoalMarkerCount() == 1) g_count_100us = 0;
            else if(side_sensor.GetGoalMarkerCount() >= 2) g_mode = FIRST_GOAL;
            break;


        default: break;
    }

#ifdef DEBUG_MODE
    g_tim6++;
#endif // DEBUG_MODE
}

void Loop()
{
    g_main_while_reset = 0;
    g_switch_state = rotary_switch.State();

    switch(g_switch_state)
    {
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
        case 0x03:
        case 0x04:
        case 0x05:
        default:
            HAL_Delay(SWITCH_CHANGE_INTERVAL_MS);
            if(g_main_while_reset == 1) break;

            g_mode = STANDBY;

            while(g_main_while_reset == 0) { led.Blink(1, 'W', 'X'); }
            break;
    }
}



void InitialDebug()
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

#ifdef MOTOR_DEBUG
    motor.Drive(INIT_DEBUG_MOTOR_DUTY, 0);
#endif // MOTOR_DEBUG
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