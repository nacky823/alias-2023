#include "line_sensor.hpp"

LineSensor::LineSensor() : adc_buffers_(), consecutive_adc_buffers_(), max_adc_values_(), min_adc_values_(), adc_values_(), emergency_stop_flag_(0) {}

void LineSensor::Init()
{
    if(HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buffers_, NUMBER_OF_ADC) != HAL_OK)
    {
        Error_Handler();
    }
    if(HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
    __HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, LED_COMPARE);
}

void LineSensor::StoreConsecutiveAdcBuffers()
{
    static uint8_t count = 0;

    for(int i = 0; i < NUMBER_OF_ADC; i++)
    {
        consecutive_adc_buffers_[count][i] = adc_buffers_[i];
    }
    if(count < NUMBER_OF_SAMPLE) count++;
    else count = 0;
}

void LineSensor::MergeSort(uint16_t array[], uint8_t first_index, uint8_t last_index)
{
    if(first_index >= last_index) return;

    int8_t i, left, right, middle;
    uint16_t temp_array[last_index+1];

    middle = (first_index + last_index) / 2;
    MergeSort(array, first_index, middle);
    MergeSort(array, middle+1, last_index);

    for(i = middle; i >= first_index; i--) temp_array[i] = array[i];
    for(i = middle+1; i <= last_index; i++)
    {
        temp_array[last_index - (i - (middle+1))] = array[i];
    }
    left  = first_index;
    right = last_index;
    for(i = first_index; i <= last_index; i++)
    {
        if(temp_array[left] < temp_array[right]) array[i] = temp_array[left++];
        else array[i] = temp_array[right--];
    }
}

void LineSensor::UpdateAdcValues()
{
    uint8_t i, j, emergency = 1;
    uint16_t sample_array[NUMBER_OF_SAMPLE];
    uint16_t sample_median, max, min, max_range, normal = 0;

    for(i = 0; i < NUMBER_OF_ADC; i++)
    {
        for(j = 0; j < NUMBER_OF_SAMPLE; j++)
        {
            sample_array[j] = consecutive_adc_buffers_[j][i];
        }
        MergeSort(sample_array, 0, NUMBER_OF_SAMPLE-1);
        sample_median = sample_array[INDEX_OF_SAMPLE_MEDIAN];

#ifdef DEBUG_MODE
        for(j = 0; j < NUMBER_OF_SAMPLE; j++) g_consecutive_buffers[i][j] = sample_array[j];
#endif // DEBUG_MODE

        if(sample_median < EMERGENCY_STOP_BORDER) emergency = 0;

        max = max_adc_values_[i];
        min = min_adc_values_[i];
        if(min > sample_median || min == 0) min = sample_median;
        else if(max < sample_median)        max = sample_median;
        max_adc_values_[i] = max;
        min_adc_values_[i] = min;

        max_range = max - min;
        if(max_range != 0) normal = MAX_VALUE * (sample_median - min) / max_range;
        adc_values_[i] = normal;
    }
    emergency_stop_flag_ = emergency;
}

uint8_t LineSensor::GetEmergencyStopFlag()
{
    return emergency_stop_flag_;
}

float LineSensor::LeftRightDifference()
{
    uint8_t i;
    uint32_t left = 0, right = 0;

    for(i = 0; i < HALF_NUMBER_OF_ADC; i++) left += adc_values_[i];
    for(i = HALF_NUMBER_OF_ADC; i < NUMBER_OF_ADC; i++) rigit += adc_values_[i];

#ifdef DEBUG_MODE
    g_line_left = left;  g_line_right = right;
#endif // DEBUG_MODE

    return static_cast<float>((right * LINE_SENSOR_CORRECTION) - left);
}

uint8_t LineSensor::CheckCalibration()
{
    uint8_t result = 0;
    static uint16_t count = 0;
    static uint16_t pre_max[NUMBER_OF_ADC] = {0};
    static uint16_t pre_min[NUMBER_OF_ADC] = {0};

    for(uint8_t i = 0; i > NUMBER_OF_ADC; i++)
    {
        if(pre_max[i] != max_adc_values_[i]) result = 1;
        else if(pre_min[i] != min_adc_values_[i]) result = 1;
        pre_max[i] = max_adc_values_[i];
        pre_min[i] = min_adc_values_[i];
    }

    if(result == 0)
    {
        if(count < CALIBRATION_COUNT) { count++; result = 1; }
    }
    else count = 0;
    
    return result;
}

#ifdef DEBUG_MODE
void LineSensor::MonitorArrays()
{
    for(uint8_t i = 0; i < NUMBER_OF_ADC; i++)
    {
        g_adc_buffers[i] = adc_buffers_[i];
        g_max_adc_values[i] = max_adc_values_[i];
        g_min_adc_values[i] = min_adc_values_[i];
        g_adc_values[i] = adc_values_[i];
    }
}
#endif // DEBUG_MODE