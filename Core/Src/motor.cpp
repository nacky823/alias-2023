#include "motor.hpp"

Motor::Motor() {}

void Motor::Init()
{
    if(HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) != HAL_OK)
    {
        Error_Handler(); // MOTOR_R
    }
    else if(HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4) != HAL_OK)
    {
        Error_Handler(); // MOTOR_L
    }
}

void Motor::Drive(float translation_ratio, float rotation_ratio)
{
#ifdef DEBUG_MODE
    g_translation_ratio = translation_ratio;  g_rotation_ratio = rotation_ratio;
#endif // DEBUG_MODE

    if(translation_ratio > LIMIT_TRANS_DUTY)       translation_ratio =  LIMIT_TRANS_DUTY;
    else if(translation_ratio < -LIMIT_TRANS_DUTY) translation_ratio = -LIMIT_TRANS_DUTY;

    float sum = translation_ratio + rotation_ratio;
    float excess = 0;

    if(sum > 1.0)       excess = sum - 1.0;
    else if(sum < -1.0) excess = sum + 1.0;

    translation_ratio -= excess;

#ifdef DEBUG_MODE
    g_sum_raito = sum;  g_excess_ratio = excess;  g_reduced_translation = translation_ratio;
#endif // DEBUG_MODE

    float duty_l = translation_ratio - rotation_ratio;
    float duty_r = translation_ratio + rotation_ratio;

    if(duty_l > 1.0)       duty_l =  1.0;
    else if(duty_l < -1.0) duty_l = -1.0;

    if(duty_r > 1.0)       duty_r =  1.0;
    else if(duty_r < -1.0) duty_r = -1.0;

#ifdef DEBUG_MODE
    g_duty_l = duty_l;  g_duty_r = duty_r;
#endif // DEBUG_MODE

    int16_t count_l = static_cast<int16_t>(COUNTER_PERIOD * duty_l);
    int16_t count_r = static_cast<int16_t>(COUNTER_PERIOD * duty_r);

    if(count_l < 0) { count_l *= -1; BACKWARD_L; }
    else if(count_l >= 0) FORWARD_L;

    if(count_r < 0) { count_r *= -1; BACKWARD_R; }
    else if(count_r >= 0) FORWARD_R;

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, count_l);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, count_r);

#ifdef DEBUG_MODE
    g_motor_compare_l = count_l;  g_motor_compare_r = count_r;
#endif // DEBUG_MODE
}