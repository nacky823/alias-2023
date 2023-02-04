#include "line_sensor.hpp"

LineSensor::LineSensor() : adc_buffers_(), consecutive_adc_buffers_(), max_adc_values_(), min_adc_values_(), emergency_stop_flag_(0) {}

void LineSensor::Init()
{
    if(HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buffers_, NUMBER_OF_ADC) != HAL_OK)
    {
        Error_Handler();
    }
    /*
    if(HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
    __HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, LED_COMPARE);
    */
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

float LineSensor::LeftRightDifference()
{
    uint8_t i, j, flag = 1;
    uint16_t sample_array[NUMBER_OF_SAMPLE];
    uint16_t sample_median, max, min, max_range;
    uint16_t normal = 0, left = 0, right = 0;

    for(i = 0; i < NUMBER_OF_ADC; i++)
    {
        for(j = 0; j < NUMBER_OF_SAMPLE; j++)
        {
            sample_array[j] = consecutive_adc_buffers_[j][i];
        }
        MergeSort(sample_array, 0, NUMBER_OF_SAMPLE-1);
        sample_median = sample_array[INDEX_OF_SAMPLE_MEDIAN];

#ifdef DEBUG_MODE
        for(j = 0; j < NUMBER_OF_SAMPLE; j++) monitor_consecutive_buffers[i][j] = sample_array[j];
#endif // DEBUG_MODE

        if(sample_median < EMERGENCY_STOP_BORDER) flag = 0;

        max = max_adc_values_[i];
        min = min_adc_values_[i];
        if(min > sample_median || min == 0) min = sample_median;
        else if(max < sample_median)        max = sample_median;
        max_adc_values_[i] = max;
        min_adc_values_[i] = min;

        max_range = max - min;
        if(max_range != 0) normal = MAX_VALUE * (sample_median - min) / max_range;
        line_sensor_values_[i] = normal;

        if(i >= 0 && i < HALF_NUMBER_OF_ADC) left += normal;
        else if(i >= HALF_NUMBER_OF_ADC && i < NUMBER_OF_ADC) right += normal;

#ifdef DEBUG_MODE
        progress_left_right_difference++;
#endif // DEBUG_MODE
    }
    emergency_stop_flag_ = flag;

#ifdef DEBUG_MODE
    monitor_line_sensor_left  = left;
    monitor_line_sensor_right = right;
#endif // DEBUG_MODE

    return right * LINE_SENSOR_CORRECTION - left;
}

uint8_t LineSensor::GetEmergencyStopFlag()
{
    return emergency_stop_flag_;
}

float LineSensor::Difference()
{
    uint16_t left = 0;
    uint16_t right = 0;
    uint8_t i;
    float diff = 0.0;

    for(i = 0, i < HALF_NUMBER_OF_ADC, i++) left += line_sensor_values_[i];
    for(i = HALF_NUMBER_OF_ADC, i < NUMBER_OF_ADC, i++) rigit += line_sensor_values_[i];

#ifdef DEBUG_MODE
    g_new_left = left;
    g_new_right = right;
#endif // DEBUG_MODE

    diff = static_cast<float>(right - left);

    return diff;
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
        monitor_adc_buffers[i] = adc_buffers_[i];
        monitor_max_adc_values[i] = max_adc_values_[i];
        monitor_min_adc_values[i] = min_adc_values_[i];
        monitor_line_sensor_values[i] = line_sensor_values_[i];
    }
}
#endif // DEBUG_MODE