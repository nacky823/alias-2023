#include "encoder.hpp"

Encoder::Encoder() : distance_(0), distance_10mm_(0), total_distance_(0) {}

void Encoder::Init()
{
	HAL_TIM_Encoder_Start(&htim8,TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL);
}

void Encoder::Update()
{
	int16_t count_l = static_cast<int16_t>((TIM8 -> CNT) - START_COUNT);
	int16_t count_r = static_cast<int16_t>(START_COUNT - (TIM4 -> CNT));
	TIM8 -> CNT = START_COUNT;
	TIM4 -> CNT = START_COUNT;
	float distance = static_cast<float>(DISTANCE_PER_COUNT * (count_l + count_r) / 2.0);

	distance_ = distance;
	distance_10mm_  += distance;
	total_distance_ += distance;

#ifdef DEBUG_MODE
	g_enc_cnt_l = count_l;  g_enc_cnt_r = count_r;
#endif // DEBUG_MODE
}

void Encoder::Reset()
{
	TIM8 -> CNT = START_COUNT;
	TIM4 -> CNT = START_COUNT;
	distance_ = 0.0;
	distance_10mm_ = 0.0;
	total_distance_ = 0.0;
}

void Encoder::ResetDistance10mm()
{
	distance_10mm_ = 0.0;
}

float Encoder::GetDistance()
{
	return distance_;
}

float Encoder::GetDistance10mm()
{
	return distance_10mm_;
}

float Encoder::GetTotalDistance()
{
	return total_distance_;
}