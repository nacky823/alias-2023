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
            if(g_tim6_complete == 0) g_tim6_yet = 0x01;
            g_tim6_complete = 0;
            line_sensor.UpdateAdcValues();
            encoder.Update();
            g_rotat = line_trace.PidControl(LINE_KP_1, LINE_KI_1, LINE_KD_1);
            g_trans = velocity_control.PidControl(TARGET_V_1, V_KP_1, V_KI_1, V_KD_1);
            motor.Drive(g_trans, g_rotat);
            EmergencyStop();
            side_sensor.IgnoreJudgment();
            g_goal_cnt = side_sensor.GetGoalMarkerCount();
            if(g_goal_cnt >= 2) g_mode = FIRST_GOAL;
            break;

        default: break;
    }
    g_tim6_complete = 1;

#ifdef DEBUG_MODE
    g_tim6++;
#endif // DEBUG_MODE
}


void InterruptTim5()
{
    switch(g_mode)
    {
        case FIRST_RUN:
            if(g_tim5_complete == 0) g_tim5_yet = 0x02;
            g_tim5_complete = 0;
            iim_42652.Update();
            uint8_t process_complete = 0;
            process_complete |= g_tim6_yet;
            process_complete |= g_tim5_yet;
            process_complete |= g_tim2_yet;
            if(g_goal_cnt == 1) Logging(process_complete);
            g_tim6_yet = 0;
            g_tim5_yet = 0;
            g_tim2_yet = 0;
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
    switch(g_mode)
    {
        case FIRST_RUN:
            if(g_tim2_complete == 0) g_tim2_yet = 0x02;
            g_tim2_complete = 0;
            g_store_periodic_log = logger.StorePeriodicLog();
            g_store_accel_log = logger.StoreAccelPositionLog();
            if(g_store_periodic_log + g_store_accel_log != 0) g_store_log_error = 1;
            else g_store_log_error = 0;
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

            led.Blink(3, 'B', 'X');
            g_mode = VELOCITY_CONTROL_DEBUG;

            while(g_main_while_reset == 0) {}
            break;

        case 0x0D: // Line trace debug
            HAL_Delay(SWITCH_CHANGE_INTERVAL_MS);
            if(g_main_while_reset == 1) break;

            led.Blink(3, 'C', 'X');
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
            g_tim6_complete = 1;
            g_mode = FIRST_RUN;

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


#ifdef DEBUG_MODE
uint8_t FlashTest()
{
    uint32_t num_of_data = 3;
    uint8_t a[num_of_data] = {1,2,3};
    uint16_t b[num_of_data] = {4,5,6};
    int16_t c[num_of_data] = {-7,-8,-9};
    float d[num_of_data] = {0.1,0.2,0.3};
    uint8_t buff_a[num_of_data];
    uint16_t buff_b[num_of_data];
    int16_t buff_c[num_of_data];
    float buff_d[num_of_data];

    if(!flash.Clear()) return 0x01;

    if(!flash.CheckBlankByte(SECTOR_1_ADDRESS_HEAD, num_of_data)) return 0x02;
    if(!flash.StoreUint8(SECTOR_1_ADDRESS_HEAD, &a, num_of_data)) return 0x03;
    if(!flash.Load(buff_a, SECTOR_1_ADDRESS_HEAD, num_of_data)) return 0x04;
    for(i = 0; i < num_of_data; i++) if(a[i] != buff_a[i]) return 0x05;

    if(!flash.CheckBlankHalfword(SECTOR_2_ADDRESS_HEAD, num_of_data)) return 0x06;
    if(!flash.StoreUint16(SECTOR_2_ADDRESS_HEAD, &b, num_of_data)) return 0x07;
    if(!flash.Load(buff_b, SECTOR_2_ADDRESS_HEAD, num_of_data*2)) return 0x08;
    for(i = 0; i < num_of_data; i++) if(b[i] != buff_b[i]) return 0x09;

    if(!flash.CheckBlankHalfword(SECTOR_3_ADDRESS_HEAD, num_of_data)) return 0x0A;
    if(!flash.StoreInt16(SECTOR_3_ADDRESS_HEAD, &c, num_of_data)) return 0x0B;
    if(!flash.Load(buff_c, SECTOR_3_ADDRESS_HEAD, num_of_data*2)) return 0x0C;
    for(i = 0; i < num_of_data; i++) if(c[i] != buff_c[i]) return 0x0D;

    if(!flash.CheckBlankWord(SECTOR_4_ADDRESS_HEAD, num_of_data)) return 0x0E;
    if(!flash.StoreFloat(SECTOR_4_ADDRESS_HEAD, &d, num_of_data)) return 0x0F;
    if(!flash.Load(buff_d, SECTOR_4_ADDRESS_HEAD, num_of_data*4)) return 0x10;
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