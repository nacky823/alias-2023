#include "iim_42652.hpp"
#include "declare_extern.h"

Iim42652::Iim42652() : degree_stack_z_(0) {}

uint8_t Iim42652::Init()
{
    /* bank_select */
    Write(REG_BANK_SEL_ADD, REG_BANK_SEL_RES, 'L');
    HAL_Delay(100); // wait 100ms
    Write(REG_BANK_SEL_ADD, REG_BANK_SEL_RES, 'R');
    HAL_Delay(100); // wait 100ms
    /* power_off */
    Write(PWR_MGMT0_ADD, PWR_MGMT0_RES, 'L');
    HAL_Delay(100); // wait 100ms
    Write(PWR_MGMT0_ADD, PWR_MGMT0_RES, 'R');
    HAL_Delay(100); // wait 100ms

    uint8_t who_l  = Read(WHO_AM_I_ADD, 'L');
    uint8_t who_r  = Read(WHO_AM_I_ADD, 'R');
    uint8_t bank_l = Read(REG_BANK_SEL_ADD, 'L');
    uint8_t bank_r = Read(REG_BANK_SEL_ADD, 'R');
    uint8_t pwr_l  = Read(PWR_MGMT0_ADD, 'L');
    uint8_t pwr_r  = Read(PWR_MGMT0_ADD, 'R');

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

    /* power_on */
    Write(PWR_MGMT0_ADD, PWR_MGMT0_ON, 'L');
    HAL_Delay(100); // wait 100ms
    Write(PWR_MGMT0_ADD, PWR_MGMT0_ON, 'R');
    HAL_Delay(100); // wait 100ms
    pwr_l = Read(PWR_MGMT0_ADD, 'L');
    pwr_r = Read(PWR_MGMT0_ADD, 'R');

#ifdef DEBUG_MODE
    g_imu_pwr_l  = pwr_l;  g_imu_pwr_r  = pwr_r;
#endif // DEBUG_MODE

    if(pwr_l != PWR_MGMT0_ON) return 0x07;
    if(pwr_r != PWR_MGMT0_ON) return 0x08;

    return 0x09;
}

uint8_t Iim42652::Read(uint8_t send_address, char imu_ic_lr)
{
    uint8_t receive_reset_value = 0x00;

    send_address = send_address | 0x80; // Read 1--- ----

    if(imu_ic_lr == 'L') CS_RESET_L;
    else if(imu_ic_lr == 'R') CS_RESET_R;

    HAL_SPI_Transmit(&hspi3, &send_address, 1, 100);
    HAL_SPI_Receive(&hspi3, &receive_reset_value, 1, 100);

    if(imu_ic_lr == 'L') CS_SET_L;
    else if(imu_ic_lr == 'R') CS_SET_R;
    
    return receive_reset_value;
}

void Iim42652::Write(uint8_t send_address, uint8_t send_data, char imu_ic_lr)
{
    send_address = send_address & 0x7F; // Write 0--- ----

    if(imu_ic_lr == 'L') CS_RESET_L;
    else if(imu_ic_lr == 'R') CS_RESET_R;

    HAL_SPI_Transmit(&hspi3, &send_address, 1, 100);
    HAL_SPI_Transmit(&hspi3, &send_data, 1, 100);

    if(imu_ic_lr == 'L') CS_SET_L;
    else if(imu_ic_lr == 'R') CS_SET_R;
}

bool Iim42652::CheckRead(uint8_t send_address, uint8_t refer_reset_value, char imu_ic_lr)
{
    uint8_t receive_reset_value = 0x00;
    bool comparison_result = false;
    
    receive_reset_value = Read(send_address, imu_ic_lr);

    if(refer_reset_value == receive_reset_value) comparison_result = true;
    else comparison_result = false;

    return comparison_result;
}

bool Iim42652::CheckWrite(uint8_t send_address, uint8_t send_data, char imu_ic_lr)
{
    Write(send_address, send_data, imu_ic_lr);

    return CheckRead(send_address, send_data, imu_ic_lr);
}

int16_t Iim42652::Read2Byte(uint8_t upper_address, uint8_t lower_address, char imu_ic_lr)
{
    int16_t upper_receive, lower_receive;

    upper_receive = static_cast<int16_t>(Read(upper_address, imu_ic_lr));
    lower_receive = static_cast<int16_t>(Read(lower_address, imu_ic_lr));

    return (upper_receive << 8) | lower_receive;
}

void Iim42652::Update()
{
    int16_t raw_z_l = Read2Byte(GYRO_DATA_Z1_ADD, GYRO_DATA_Z0_ADD, 'L');
    int16_t raw_z_r = Read2Byte(GYRO_DATA_Z1_ADD, GYRO_DATA_Z0_ADD, 'R');
    double avg_raw_z = static_cast<double>(raw_z_l + raw_z_r) / 2.0;
    double degree_z = avg_raw_z * NORMAL_FUCOR_DPS * TIM6_PERIOD;

    degree_stack_z_ += degree_z;

#ifdef DEBUG_MODE
    g_deg_z = static_cast<float>(degree_stack_z_);
#endif // DEBUG_MODE
}

void Iim42652::ResetDegreeStackZ()
{
    degree_stack_z_ = 0.0;
}

double Iim42652::GetDegreeStackZ()
{
    return degree_stack_z_;
}

int16_t Iim42652::GyroXLeft()
{
    return Read2Byte(GYRO_DATA_X1_ADD, GYRO_DATA_X0_ADD, 'L');
}

int16_t Iim42652::GyroXRight()
{
    return Read2Byte(GYRO_DATA_X1_ADD, GYRO_DATA_X0_ADD, 'R');
}

int16_t Iim42652::GyroYLeft()
{
    return Read2Byte(GYRO_DATA_Y1_ADD, GYRO_DATA_Y0_ADD, 'L');
}

int16_t Iim42652::GyroYRight()
{
    return Read2Byte(GYRO_DATA_Y1_ADD, GYRO_DATA_Y0_ADD, 'R');
}

int16_t Iim42652::GyroZLeft()
{
    return Read2Byte(GYRO_DATA_Z1_ADD, GYRO_DATA_Z0_ADD, 'L');
}

int16_t Iim42652::GyroZRight()
{
    return Read2Byte(GYRO_DATA_Z1_ADD, GYRO_DATA_Z0_ADD, 'R');
}

int16_t Iim42652::AccelXLeft()
{
    return Read2Byte(ACCEL_DATA_X1_ADD, ACCEL_DATA_X0_ADD, 'L');
}

int16_t Iim42652::AccelXRight()
{
    return Read2Byte(ACCEL_DATA_X1_ADD, ACCEL_DATA_X0_ADD, 'R');
}

int16_t Iim42652::AccelYLeft()
{
    return Read2Byte(ACCEL_DATA_Y1_ADD, ACCEL_DATA_Y0_ADD, 'L');
}

int16_t Iim42652::AccelYRight()
{
    return Read2Byte(ACCEL_DATA_Y1_ADD, ACCEL_DATA_Y0_ADD, 'R');
}

int16_t Iim42652::AccelZLeft()
{
    return Read2Byte(ACCEL_DATA_Z1_ADD, ACCEL_DATA_Z0_ADD, 'L');
}

int16_t Iim42652::AccelZRight()
{
    return Read2Byte(ACCEL_DATA_Z1_ADD, ACCEL_DATA_Z0_ADD, 'R');
}