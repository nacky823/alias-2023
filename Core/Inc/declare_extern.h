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
extern uint16_t monitor_adc_buffers[NUMBER_OF_ADC];
extern uint16_t monitor_consecutive_buffers[NUMBER_OF_ADC][NUMBER_OF_SAMPLE];
extern uint16_t monitor_max_adc_values[NUMBER_OF_ADC];
extern uint16_t monitor_min_adc_values[NUMBER_OF_ADC];
extern uint16_t monitor_line_sensor_values[NUMBER_OF_ADC];
extern uint16_t progress_left_right_difference;
extern uint16_t monitor_line_sensor_left;
extern uint16_t monitor_line_sensor_right;
extern uint8_t g_ignore_flag;
extern uint8_t g_pre_noise_state_cntup;
extern uint8_t g_current_state_cntup;
extern uint8_t g_exception_flags_cntup;
extern uint8_t g_master_count_cntup;
extern uint8_t g_exception_flags_cross;
extern uint8_t g_master_count_cross;
extern uint8_t g_write_state_cntup;
extern uint8_t g_goal_reach_cntup;
extern uint8_t g_corner_reach_cntup;
extern uint8_t g_black_flag_cntup;
extern uint8_t g_goal_flag_cntup;
extern uint8_t g_corner_flag_cntup;
extern uint8_t g_cross_flag_cntup;
extern uint8_t g_write_state_cntup_2;
extern uint8_t g_goal_reach_cntup_2;
extern uint8_t g_corner_reach_cntup_2;
extern uint8_t g_black_flag_cntup_2;
extern uint8_t g_goal_flag_cntup_2;
extern uint8_t g_corner_flag_cntup_2;
extern uint8_t g_cross_flag_cntup_2;
extern float monitor_p;
extern float monitor_d;
extern float monitor_i;
extern float g_distance;
extern int16_t g_count_enc_l;
extern int16_t g_count_enc_r;
extern float g_translation_ratio;
extern float g_rotation_ratio;
extern float g_sum;
extern float g_excess;
extern float g_reduced_translation;
extern float g_duty_l;
extern float g_duty_r;
extern int16_t g_count_l;
extern int16_t g_count_r;
#endif // DEBUG_MODE

#endif // DECLARE_EXTERN_H_