#ifndef DECLARE_EXTERN_H_
#define DECLARE_EXTERN_H_

#include "macro.h"
#include <stdbool.h>

extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;

extern I2C_HandleTypeDef hi2c2;

extern SPI_HandleTypeDef hspi3;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim11;
extern TIM_HandleTypeDef htim12;

#ifdef DEBUG_MODE
/* Anywhere */
extern uint16_t monitor_count;

/*
#ifdef DEBUG_MODE
monitor_count++;
#endif // DEBUG_MODE
*/
extern bool g_error_handler_tim1_ch2;
extern bool g_error_handler_tim1_ch4;
extern bool g_error_handler_tim4;
extern bool g_error_handler_tim8;
extern bool g_interval_enable;

/* Line sensor */
extern float g_line_diff;
extern uint8_t g_line_emer;

/* Side seneor */
extern uint8_t g_ignore_flag;
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
extern uint8_t g_imu_who_l, g_imu_who_r;
extern uint8_t g_imu_bank_l, g_imu_bank_r;
extern uint8_t g_imu_pwr_l, g_imu_pwr_r;
extern uint8_t g_imu_init;
extern double g_deg_stack_z;
extern double g_deg_z;
extern int16_t g_gyro_x_l, g_gyro_x_r;
extern int16_t g_gyro_y_l, g_gyro_y_r;
extern int16_t g_gyro_z_l, g_gyro_z_r;
extern int16_t g_accel_x_l, g_accel_x_r;
extern int16_t g_accel_y_l, g_accel_y_r;
extern int16_t g_accel_z_l, g_accel_z_r;

/* Encoder */
extern int16_t g_enc_cnt_l, g_enc_cnt_r;
extern float g_distance_l, g_distance_r;
extern float g_distance;
extern float g_distance_stack;
extern float g_distance_diff;

/* Motor */
extern float g_translation_ratio;
extern float g_rotation_ratio;
extern float g_sum_raito;
extern float g_excess_ratio;
extern float g_reduced_translation;
extern float g_duty_l, g_duty_r;
extern int16_t g_motor_compare_l, g_motor_compare_r;

/* Flash */
extern uint32_t g_erase_failed_sector;

/* Line trace */
extern float g_trace_p;
extern float g_trace_i;
extern float g_trace_d;

/* Velocity control */
extern float g_target_velocity;
extern float g_current_velocity;
extern float g_filter_velocity;
extern float g_velocity_error;
extern float g_velocity_p, g_velocity_i, g_velocity_d;
extern float g_velocity_p_gain, g_velocity_i_gain, g_velocity_d_gain;

/* Wrapper */
extern uint8_t g_imu_init;
extern uint32_t g_external_interrupt;
extern uint32_t g_tim7;
extern uint32_t g_tim6;
extern uint32_t g_tim5;
extern uint32_t g_tim2;
extern uint8_t g_flash_test;
#endif // DEBUG_MODE

/* Wrapper */
extern uint8_t g_main_while_reset;
extern uint8_t g_switch_state;
extern uint8_t g_mode;
extern uint8_t g_emergency_stop;
extern uint8_t g_run_end;
/* Timer */
extern uint8_t g_tim6_complete, g_tim5_complete, g_tim2_complete;
extern uint8_t g_tim6_yet, g_tim5_yet, g_tim2_yet;
extern uint8_t g_line_calib;
extern uint8_t g_goal_cnt;
extern float g_trans;
extern float g_rotat;
extern uint8_t g_first_log_failed;
extern uint8_t g_store_periodic_log;
extern uint8_t g_store_accel_log;
extern float g_target;
extern float g_common_speed;
extern uint8_t g_process_complete;
extern uint8_t g_flash_erase;

#endif // DECLARE_EXTERN_H_