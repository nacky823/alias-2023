#include "encoder.hpp"

Encoder::Encoder() : distance_(0)
				   , distance_stack_(0)
				   , distance_difference_(0) {}

void Encoder::Init()
{
	if(HAL_TIM_Encoder_Start(&htim8,TIM_CHANNEL_ALL) != HAL_OK)
	{
		Error_Handler();
	}

	if(HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL) != HAL_OK)
	{
		Error_Handler();
	}
}

void Encoder::Update()
{
	int16_t count_l = static_cast<int16_t>((TIM8 -> CNT) - START_COUNT);
	int16_t count_r = static_cast<int16_t>(START_COUNT - (TIM4 -> CNT));
	TIM8 -> CNT = START_COUNT;
	TIM4 -> CNT = START_COUNT;
	float distance_l = static_cast<float>(count_l * DISTANCE_PER_COUNT);
	float distance_r = static_cast<float>(count_r * DISTANCE_PER_COUNT);
	float distance = (distance_l + distance_r) / 2.0;

	distance_ = distance;
	distance_stack_ += distance;
	distance_difference_ = distance_r - distance_l;

#ifdef DEBUG_MODE
	g_enc_cnt_l = count_l;  g_enc_cnt_r = count_r;
	g_distance_l = distance_l;  g_distance_r = distance_r;
#endif // DEBUG_MODE
}

void Encoder::Reset()
{
	distance_ = 0.0;
	distance_stack_ = 0.0;
	distance_difference_ = 0.0;
	TIM8 -> CNT = START_COUNT;
	TIM4 -> CNT = START_COUNT;
}

void Encoder::ResetDistanceStack()
{
	distance_stack_ = 0.0;
}

float Encoder::GetDistance()
{
	return distance_;
}

float Encoder::GetDistanceStack()
{
	return distance_stack_;
}

float Encoder::AngularVelocity()
{
	return distance_difference_ / (TIM6_PERIOD * MACHINE_TREAD);
}