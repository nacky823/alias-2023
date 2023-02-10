#ifndef IIM_42652_HPP_
#define IIM_42652_HPP_

#include "stm32f4xx_hal.h"
#include "declare_extern.h"

#define CS_RESET_L HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET)
#define CS_SET_L   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET)
#define CS_RESET_R HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET)
#define CS_SET_R   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET)

/* -------------------- BANK 0 REGISTER -------------------- */

/* who_am_i */
#define WHO_AM_I_ADD 0x75
#define WHO_AM_I_RES 0x6F
/* bank_select */
#define REG_BANK_SEL_ADD 0x76
#define REG_BANK_SEL_RES 0x00 // BANK 0
/* power_on_off */
#define PWR_MGMT0_ADD 0x4E
#define PWR_MGMT0_RES 0x00 // Gyro and Accel OFF
#define PWR_MGMT0_ON  0x0F // Gyro and Accel ON
/* accel_data_x */
#define ACCEL_DATA_X1_ADD 0x1F
#define ACCEL_DATA_X1_RES 0x80
#define ACCEL_DATA_X0_ADD 0x20
#define ACCEL_DATA_X0_RES 0x00
/* accel_data_y */
#define ACCEL_DATA_Y1_ADD 0x21
#define ACCEL_DATA_Y1_RES 0x80
#define ACCEL_DATA_Y0_ADD 0x22
#define ACCEL_DATA_Y0_RES 0x00
/* accel_data_z */
#define ACCEL_DATA_Z1_ADD 0x23
#define ACCEL_DATA_Z1_RES 0x80
#define ACCEL_DATA_Z0_ADD 0x24
#define ACCEL_DATA_Z0_RES 0x00
/* gyro_data_x */
#define GYRO_DATA_X1_ADD 0x25
#define GYRO_DATA_X1_RES 0x80
#define GYRO_DATA_X0_ADD 0x26
#define GYRO_DATA_X0_RES 0x00
/* gyro_data_y */
#define GYRO_DATA_Y1_ADD 0x27
#define GYRO_DATA_Y1_RES 0x80
#define GYRO_DATA_Y0_ADD 0x28
#define GYRO_DATA_Y0_RES 0x00
/* gyro_data_z */
#define GYRO_DATA_Z1_ADD 0x29
#define GYRO_DATA_Z1_RES 0x80
#define GYRO_DATA_Z0_ADD 0x2A
#define GYRO_DATA_Z0_RES 0x00
/* fs_sel, odr */
#define GYRO_CONFIG0_ADD 0x4F
#define GYRO_CONFIG0_RES 0x06
#define ACCEL_CONFIG0_ADD 0x50
#define ACCEL_CONFIG0_RES 0x06
/* spi_slew_rate */
#define DRIVE_CONFIG_ADD 0x13
#define DRIVE_CONFIG_RES 0x05
/* spi_mode, soft_reset */
#define DEVICE_CONFIG_ADD 0x11
#define DEVICE_CONFIG_RES 0x00
/* clock */
#define INTF_CONFIG1_ADD 0x4D
#define INTF_CONFIG1_RES 0x91
/* ui_filt_ord */
#define GYRO_CONFIG1_ADD 0x51
#define GYRO_CONFIG1_RES 0x16
#define ACCEL_CONFIG1_ADD 0x53
#define ACCEL_CONFIG1_RES 0x0D
/* ui_filt_bw */
#define GYRO_ACCEL_CONFIG0_ADD 0x52
#define GYRO_ACCEL_CONFIG0_RES 0x11
/* time_stamp */
#define TMST_CONFIG_ADD 0x54
#define TMST_CONFIG_RES 0x23
/* self_test_on_off */
#define SELF_TEST_CONFIG_ADD 0x70
#define SELF_TEST_CONFIG_RES 0x00
/* FIFO */
#define FIFO_CONFIG_ADD 0x16
#define FIFO_CONFIG_RES 0x00
#define INT_STATUS_ADD 0x2D
#define INT_STATUS_RES 0x10
#define FIFO_COUNTH_ADD 0x2E
#define FIFO_COUNTH_RES 0x00
#define FIFO_COUNTL_ADD 0x2F
#define FIFO_COUNTL_RES 0x00
#define FIFO_DATA_ADD 0x30
#define FIFO_DATA_RES 0xFF
#define SIGNAL_PATH_RESET_ADD 0x4B
#define INTF_CONFIG0_ADD 0x4C
#define INTF_CONFIG0_RES 0x30
#define FIFO_CONFIG1_ADD 0x5F
#define FIFO_CONFIG1_RES 0x00
#define FIFO_CONFIG2_ADD 0x60
#define FIFO_CONFIG2_RES 0x00
#define FIFO_CONFIG3_ADD 0x61
#define FIFO_CONFIG3_RES 0x00
#define FIFO_LOST_PKT0_ADD 0x6C
#define FIFO_LOST_PKT0_RES 0x00
#define FIFO_LOST_PKT1_ADD 0x6D
#define FIFO_LOST_PKT1_RES 0x00

/* -------------------- BANK 1 REGISTER -------------------- */

/* sensor on off */
#define SENSOR_CONFIG0_ADD 0x03
#define SENSOR_CONFIG0_RES 0x80
/* anti-alias */
#define GYRO_CONFIG_STATIC2_ADD 0x0B
#define GYRO_CONFIG_STATIC2_RES 0xA0
#define GYRO_CONFIG_STATIC3_ADD 0x0C
#define GYRO_CONFIG_STATIC3_RES 0x0D
#define GYRO_CONFIG_STATIC4_ADD 0x0D
#define GYRO_CONFIG_STATIC4_RES 0xAA
#define GYRO_CONFIG_STATIC5_ADD 0x0E
#define GYRO_CONFIG_STATIC5_RES 0x80
/* notch x axis factory trimmed */
#define GYRO_CONFIG_STATIC6_ADD 0x0F
#define GYRO_CONFIG_STATIC6_FAC 0x7D // decimal 125
/* notch y axis factory trimmed */
#define GYRO_CONFIG_STATIC7_ADD 0x10
#define GYRO_CONFIG_STATIC7_FAC 0x92 // decimal 146
/* notch z axis factory trimmed */
#define GYRO_CONFIG_STATIC8_ADD 0x11
#define GYRO_CONFIG_STATIC8_FAC 0x80 // decimal 128
/* notch xyz factory trimmed */
#define GYRO_CONFIG_STATIC9_ADD 0x12
#define GYRO_CONFIG_STATIC9_FAC 0x38 // decimal 56
/* notch, high pass */
#define GYRO_CONFIG_STATIC10_ADD 0x13
#define GYRO_CONFIG_STATIC10_RES 0x11
/* self test data gx factory trimmed */
#define XG_ST_DATA_ADD 0x5F
#define XG_ST_DATA_FAC 0xAE // decimal 174
/* self test data gy factory trimmed */
#define YG_ST_DATA_ADD 0x60
#define YG_ST_DATA_FAC 0xB2 // decimal 178
/* self test data gz factory trimmed */
#define ZG_ST_DATA_ADD 0x61
#define ZG_ST_DATA_FAC 0xB0 // decimal 176
/* time_stamp */
#define TMSTVAL0_ADD 0x62
#define TMSTVAL0_RES 0x00
#define TMSTVAL1_ADD 0x63
#define TMSTVAL1_RES 0x00
#define TMSTVAL2_ADD 0x64
#define TMSTVAL2_RES 0x00
/* spi_ap_4_wire */
#define INTF_CONFIG4_ADD 0x7A
#define INTF_CONFIG4_RES 0x03

/* -------------------- BANK 2 REGISTER -------------------- */

/* anti-alias */
#define ACCEL_CONFIG_STATIC2_ADD 0x03
#define ACCEL_CONFIG_STATIC2_RES 0x30
#define ACCEL_CONFIG_STATIC3_ADD 0x04
#define ACCEL_CONFIG_STATIC3_RES 0x40
#define ACCEL_CONFIG_STATIC4_ADD 0x05
#define ACCEL_CONFIG_STATIC4_RES 0x62
/* self test data ax factory trimmed */
#define XA_ST_DATA_ADD 0x3B
#define XA_ST_DATA_FAC 0x21 // decimal 33
/* self test data ax factory trimmed */
#define YA_ST_DATA_ADD 0x3C
#define YA_ST_DATA_FAC 0x31 // decimal 49
/* self test data ax factory trimmed */
#define ZA_ST_DATA_ADD 0x3D
#define ZA_ST_DATA_FAC 0x96 // decimal 150

/* -------------------- BANK 4 REGISTER -------------------- */

/* FIFO */
#define FDR_CONFIG_ADD 0x09
#define FDR_CONFIG_RES 0x00

class Iim42652
{
private:
    double degree_stack_z_;

public:
    Iim42652();
    uint8_t Init();
    uint8_t Read(uint8_t, char);
    void Write(uint8_t, uint8_t, char);
    bool CheckRead(uint8_t, uint8_t, char);
    bool CheckWrite(uint8_t, uint8_t, char);
    int16_t Read2Byte(uint8_t, uint8_t, char);
    void Update();
    void ResetDegreeStackZ();
    double GetDegreeStackZ();
    int16_t GyroXLeft();
    int16_t GyroXRight();
    int16_t GyroYLeft();
    int16_t GyroYRight();
    int16_t GyroZLeft();
    int16_t GyroZRight();
    int16_t AccelXLeft();
    int16_t AccelXRight();
    int16_t AccelYLeft();
    int16_t AccelYRight();
    int16_t AccelZLeft();
    int16_t AccelZRight();

};

#endif // IIM_42652_HPP_