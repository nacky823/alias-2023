#include "encoder.hpp"

Encoder::Encoder() : count_l_(0), count_r_(0), distance_(0), total_distance_(0) {}

void Encoder::Init()
{
	HAL_TIM_Encoder_Start(&htim8,TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL);
}

void Encoder::ResetCountDistance()
{
	count_l_ = 0;
	count_r_ = 0;
	distance_ = 0;
}

void Encoder::UpdateCountDistance()
{
	count_l_ = static_cast<int16_t>((TIM8 -> CNT) - START_COUNT);
	count_r_ = static_cast<int16_t>(START_COUNT - (TIM4 -> CNT));
	TIM8 -> CNT = START_COUNT;
	TIM4 -> CNT = START_COUNT;
	distance_ = DISTANCE_PER_COUNT * static_cast<float>(count_l_ + count_r_) / 2.0;
}

void Encoder::AddTotalDistance()
{
	total_distance_ += distance_;
}

float Encoder::GetDistance()
{
	return distance_;
}

float Encoder::GetTotalDistance()
{
	return total_distance_;
}

#ifdef DEBUG_MODE
void Encoder::GetCount(int16_t &count_l, int16_t &count_r)
{
	count_l = count_l_;
	count_r = count_r_;
}
#endif // DEBUG_MODE