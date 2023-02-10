#ifndef DECLARE_EXTERN_H_
#define DECLARE_EXTERN_H_

#include "macro.h"

extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;

extern I2C_HandleTypeDef hi2c2;

extern SPI_HandleTypeDef hspi3;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim11;
extern TIM_HandleTypeDef htim12;
extern TIM_HandleTypeDef htim14;

#ifdef DEBUG_MODE

/* Line sensor */
extern uint16_t g_adc_buffers[NUMBER_OF_ADC];
extern uint16_t g_consecutive_buffers[NUMBER_OF_ADC][NUMBER_OF_SAMPLE];
extern uint16_t g_max_adc_values[NUMBER_OF_ADC];
extern uint16_t g_min_adc_values[NUMBER_OF_ADC];
extern uint16_t g_adc_values[NUMBER_OF_ADC];
extern uint32_t g_line_left;
extern uint32_t g_line_right;
extern float g_line_diff;
extern uint8_t g_line_emer;
extern uint8_t g_line_calib;

/* Side seneor */
extern uint8_t g_ignore_flag;
extern uint8_t g_goal_cnt;
extern uint8_t g_corner_cnt;
extern uint8_t g_cross_cnt;
/* Monitor method CountUp 1 */
extern uint8_t g_pre_noise_state_cntup;
extern uint8_t g_current_state_cntup;
extern uint8_t g_exception_flags_cntup;
extern uint8_t g_master_count_cntup;
/* Monitor method CountUp 2 */
extern uint8_t g_exception_flags_cross;
extern uint8_t g_master_count_cross;
extern uint8_t g_write_state_cntup;
extern uint8_t g_goal_reach_cntup;
extern uint8_t g_corner_reach_cntup;
extern uint8_t g_black_flag_cntup;
extern uint8_t g_goal_flag_cntup;
extern uint8_t g_corner_flag_cntup;
extern uint8_t g_cross_flag_cntup;
/* Monitor method CountUp 3 */
extern uint8_t g_write_state_cntup_2;
extern uint8_t g_goal_reach_cntup_2;
extern uint8_t g_corner_reach_cntup_2;
extern uint8_t g_black_flag_cntup_2;
extern uint8_t g_goal_flag_cntup_2;
extern uint8_t g_corner_flag_cntup_2;
extern uint8_t g_cross_flag_cntup_2;

/* IMU */
extern uint8_t g_imu_who_l;
extern uint8_t g_imu_who_r;
extern uint8_t g_imu_bank_l;
extern uint8_t g_imu_bank_r;
extern uint8_t g_imu_pwr_l;
extern uint8_t g_imu_pwr_r;
extern uint8_t g_imu_init;
extern double g_deg_stack_z;
extern double g_deg_z;
extern int16_t g_gyro_x_l;
extern int16_t g_gyro_x_r;
extern int16_t g_gyro_y_l;
extern int16_t g_gyro_y_r;
extern int16_t g_gyro_z_l;
extern int16_t g_gyro_z_r;
extern int16_t g_accel_x_l;
extern int16_t g_accel_x_r;
extern int16_t g_accel_y_l;
extern int16_t g_accel_y_r;
extern int16_t g_accel_z_l;
extern int16_t g_accel_z_r;

/* Motor */
extern float g_translation_ratio;
extern float g_rotation_ratio;
extern float g_sum_raito;
extern float g_excess_ratio;
extern float g_reduced_translation;
extern float g_duty_l;
extern float g_duty_r;
extern int16_t g_motor_compare_l;
extern int16_t g_motor_compare_r;




extern float monitor_p;
extern float monitor_d;
extern float monitor_i;
extern float g_distance;
extern int16_t g_count_enc_l;
extern int16_t g_count_enc_r;



#endif // DEBUG_MODE

#endif // DECLARE_EXTERN_H_