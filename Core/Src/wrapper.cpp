#include "wrapper.hpp"
#include "encoder.hpp"
#include "iim_42652.hpp"
#include "led.hpp"
#include "line_sensor.hpp"
#include "motor.hpp"
#include "rotary_switch.hpp"
#include "side_sensor.hpp"
#include "declare_extern.h"

Encoder encoder;
Iim42652 iim_42652;
Led led;
LineSensor line_sensor;
Motor motor;
RotarySwitch rotary_switch;
SideSensor side_sensor;

void Init()
{
    line_sensor.Init();
}

void Interrupt100us()
{
    line_sensor.StoreConsecutiveAdcBuffers();
}

uint8_t ImuActive()
{
    uint8_t who_l  = iim_42652.Read(WHO_AM_I_ADD, 'L');
    uint8_t who_r  = iim_42652.Read(WHO_AM_I_ADD, 'R');
    uint8_t bank_l = iim_42652.Read(REG_BANK_SEL_ADD, 'L');
    uint8_t bank_r = iim_42652.Read(REG_BANK_SEL_ADD, 'R');
    uint8_t pwr_l  = iim_42652.Read(PWR_MGMT0_ADD, 'L');
    uint8_t pwr_r  = iim_42652.Read(PWR_MGMT0_ADD, 'R');

#ifdef DEBUG_MODE
    g_imu_who_l  = who_l;  g_imu_who_r  = who_r;
    g_imu_bank_l = bank_l; g_imu_bank_r = bank_r;
    g_imu_pwr_l  = pwr_l;  g_imu_pwr_r  = pwr_r;
#endif // DEBUG_MODE

    if(who_l != WHO_AM_I_RES)      return 0x01;
    if(who_r != WHO_AM_I_RES)      return 0x02;
    if(bank_l != REG_BANK_SEL_RES) return 0x03;
    if(bank_r != REG_BANK_SEL_RES) return 0x04;
    if(pwr_l != PWR_MGMT0_RES)     return 0x05;
    if(pwr_r != PWR_MGMT0_RES)     return 0x06;

    iim_42652.Write(PWR_MGMT0_ADD, PWR_MGMT0_ON, 'L');
    HAL_Delay(100); // wait 100ms
    iim_42652.Write(PWR_MGMT0_ADD, PWR_MGMT0_ON, 'R');
    HAL_Delay(100); // wait 100ms
    pwr_l = iim_42652.Read(PWR_MGMT0_ADD, 'L');
    pwr_r = iim_42652.Read(PWR_MGMT0_ADD, 'R');

#ifdef DEBUG_MODE
    g_imu_pwr_l  = pwr_l;  g_imu_pwr_r  = pwr_r;
#endif // DEBUG_MODE

    if(pwr_l != PWR_MGMT0_ON) return 0x07;
    if(pwr_r != PWR_MGMT0_ON) return 0x08;

    return 0x09;
}

#ifdef DEBUG_MODE
void Monitor()
{
    /* Encoder */
    encoder.UpdateCountDistance();
    encoder.GetCount(g_enc_cnt_l, g_enc_cnt_r);
    g_distance      = encoder.GetDistance();
    g_distance_10mm = encoder.GetDistance10mm();

    /* IMU */
    if(g_imu_active == 0x09)
    {
        g_gyro_x_l  = iim_42652.GyroXLeft();
        g_gyro_x_r  = iim_42652.GyroXRight();
        g_gyro_y_l  = iim_42652.GyroYLeft();
        g_gyro_y_r  = iim_42652.GyroYRight();
        g_gyro_z_l  = iim_42652.GyroZLeft();
        g_gyro_z_r  = iim_42652.GyroZRight();
        g_accel_x_l = iim_42652.AccelXLeft();
        g_accel_x_r = iim_42652.AccelXRight();
        g_accel_y_l = iim_42652.AccelYLeft();
        g_accel_y_r = iim_42652.AccelYRight();
        g_accel_z_l = iim_42652.AccelZLeft();
        g_accel_z_r = iim_42652.AccelZRight();
    }

    /* LED */
    uint8_t color = 0x0F;

    /* Line sensor */
    line_sensor.MonitorArrays();
    g_line_diff     = line_sensor.LeftRightDifference();
    g_new_line_diff = line_sensor.Difference();
    uint8_t line_eme = line_sensor.GetEmergencyStopFlag();
    if(line_eme == 1) color = 0x01;
    g_line_eme = line_eme;
    uint8_t line_calib = line_sensor.CheckCalibration();
    if(line_calib == 1) color = 0x00;
    g_line_calib = line_calib;

    /* Motor */
    if(rotary_switch.State() == 0x03)
    {
        motor.Drive(0.10, 0);
    }
    else motor.Drive(0, 0);
    motor.GetRatio(g_translation_ratio, g_rotation_ratio);
    motor.GetLimitValues(g_sum_raito, g_excess_ratio, g_reduced_translation);
    motor.GetDuty(g_duty_l, g_duty_r);
    motor.GetCount(g_count_l, g_count_r);

    /* Rotary switch */
    if(rotary_switch.State() == 0x00) color = 0x02;

    /* Side seneor */
    side_sensor.IgnoreJudgment();
    g_goal_cnt   = side_sensor.GetGoalMarkerCount();
    g_corner_cnt = side_sensor.GetCornerMarkerCount();
    g_cross_cnt  = side_sensor.GetCrossLineCount();

    /* LED2 */
    switch(color)
    {
        case 0x00: led.ColorOrder('R'); break;
        case 0x01: led.ColorOrder('G'); break;
        case 0x02: led.ColorOrder('B'); break;
        default:   led.ColorOrder('X'); break;
    }
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