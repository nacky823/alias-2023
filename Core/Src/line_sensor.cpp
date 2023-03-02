#include "line_sensor.hpp"

LineSensor::LineSensor() : line_sensors_buff_()
                         , consecutive_line_sensors_buff_()
                         , max_line_sensors_valu_()
                         , min_line_sensors_valu_()
                         , line_sensors_valu_()
                         , emergency_stop_flag_(false) {}

void LineSensor::Init()
{
    if(HAL_ADC_Start_DMA(&hadc1, (uint32_t *)line_sensors_buff_, NUM_OF_LINE_SENSORS) != HAL_OK)
    {
#ifdef DEBUG_MODE
        g_error_handler_adc1 = true;
#endif // DEBUG_MODE

        Error_Handler();
    }

    if(HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1) != HAL_OK)
    {
#ifdef DEBUG_MODE
        g_error_handler_tim11 = true;
#endif // DEBUG_MODE

        Error_Handler();
    }

    __HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, LINE_SENSORS_LED_COMPARE);
}

void LineSensor::StoreConsecutiveBuff()
{
    static uint8_t times = 0;

    for(int i = 0; i < NUM_OF_LINE_SENSORS; i++)
    {
        consecutive_line_sensors_buff_[times][i] = line_sensors_buff_[i];
    }

    if(times < CONSECUTIVE_TIMES) times++;
    else times = 0;
}

void LineSensor::MergeSort(uint16_t array[], uint8_t first_index, uint8_t last_index)
{
    if(first_index >= last_index) return;

    int8_t i, left, right, middle;
    uint16_t temp_array[last_index+1];

    middle = (first_index + last_index) / 2;
    MergeSort(array, first_index, middle);
    MergeSort(array, middle+1, last_index);

    for(i = middle; i >= first_index; i--)
    {
        temp_array[i] = array[i];
    }
    for(i = middle+1; i <= last_index; i++)
    {
        temp_array[last_index - (i - (middle+1))] = array[i];
    }

    left  = first_index;
    right = last_index;

    for(i = first_index; i <= last_index; i++)
    {
        if(temp_array[left] < temp_array[right]) array[i] = temp_array[left++];
        else                                     array[i] = temp_array[right--];
    }
}

void LineSensor::New()
{
    uint8_t i, j;
    uint16_t temp_array[CONSECUTIVE_TIMES];
    uint16_t sorted_array[NUM_OF_LINE_SENSORS][CONSECUTIVE_TIMES];
    
    for(i = 0; i < NUM_OF_LINE_SENSORS; i++)
    {
        for(j = 0; j < CONSECUTIVE_TIMES; j++)
        {
            temp_array[j] = consecutive_line_sensors_buff_[j][i];
        }

        MergeSort(temp_array, 0, CONSECUTIVE_TIMES-1);

        for(j = 0; j < CONSECUTIVE_TIMES; j++)
        {
            sorted_array[i][j] = temp_array[j];
            g_consecutive_line_buff[i][j] = sorted_array[i][j];
        }
    }

    for(i = 0; i < NUM_OF_LINE_SENSORS; i++)
    {
        
        g_max_line_valu[i] = max_line_sensors_valu_[i];
        g_min_line_valu[i] = min_line_sensors_valu_[i];
        g_line_valu[i] = line_sensors_valu_[i];
        
        /*
        if(min_line_sensors_valu_[i] > sorted_array[i][HALF_CONSECUTIVE_TIMES])
        {
            min_line_sensors_valu_[i] = sorted_array[i][HALF_CONSECUTIVE_TIMES];
            g_min_line_valu[i] = min_line_sensors_valu_[i];
        }
        else if(min_line_sensors_valu_[i] == 0)
        {
            min_line_sensors_valu_[i] = sorted_array[i][HALF_CONSECUTIVE_TIMES];
            g_min_line_valu[i] = min_line_sensors_valu_[i];
        }
        */
    }
}
/*
void LineSensor::Update()
{
    uint8_t i, j;
    uint16_t temp_array[CONSECUTIVE_TIMES];
    uint16_t median, max, min, normalized;
    bool emergency = true;

    for(i = 0; i < NUM_OF_LINE_SENSORS; i++)
    {
        for(j = 0; j < CONSECUTIVE_TIMES; j++)
        {
            temp_array[j] = consecutive_line_sensors_buff_[j][i];
        }

        MergeSort(temp_array, 0, CONSECUTIVE_TIMES-1);
        median = temp_array[HALF_CONSECUTIVE_TIMES];

#ifdef DEBUG_MODE
        for(j = 0; j < CONSECUTIVE_TIMES; j++)
        {
            g_consecutive_line_buff[i][j] = temp_array[j];
        }
#endif // DEBUG_MODE

        /*
        max = max_line_sensors_valu_[i];
        min = min_line_sensors_valu_[i];
        if(max < median)      max = median;
        else if(min > median) min = median;
        else if(min == 0)     min = median;
        max_line_sensors_valu_[i] = max;
        min_line_sensors_valu_[i] = min;

        max = max_line_sensors_valu_[i];
        min = min_line_sensors_valu_[i];
        if(max < median)
        {
            max = median;
            max_line_sensors_valu_[i] = median;
            g_max_line_valu[i] = max_line_sensors_valu_[i];
        }
        if(min > median || min == 0)
        {
            min = median;
            min_line_sensors_valu_[i] = median;
            g_min_line_valu[i] = min_line_sensors_valu_[i];
        }


        if(max > min)
        {
            normalized = MAX_NORMALIZED_VALU * (median - min) / (max - min);
            line_sensors_valu_[i] = normalized;
            g_line_valu[i] = line_sensors_valu_[i];

            if(normalized < EMERGENCY_STOP_BORDER) emergency = false;
        }
    }

    emergency_stop_flag_ = emergency;
}
*/

bool LineSensor::GetEmergencyStopFlag()
{
    return emergency_stop_flag_;
}

float LineSensor::LeftRightDifference()
{
    uint8_t i;
    uint32_t left = 0, right = 0;

    for(i = 0; i < HALF_NUM_OF_LINE_SENSORS; i++)
    {
        left += line_sensors_valu_[i];
    }
    for(i = HALF_NUM_OF_LINE_SENSORS; i < NUM_OF_LINE_SENSORS; i++)
    {
        right += line_sensors_valu_[i];
    }

#ifdef DEBUG_MODE
    g_line_left = left;
    g_line_right = right;
#endif // DEBUG_MODE

    float difference = right * LINE_SENSOR_CORRECTION - left;

    return difference;
}

bool LineSensor::CheckCalibration()
{
    static uint16_t timer = 0;
    static uint16_t pre_max[NUM_OF_LINE_SENSORS] = {0};
    static uint16_t pre_min[NUM_OF_LINE_SENSORS] = {0};
    uint16_t max, min;
    bool result = true;

    for(uint8_t i = 0; i > NUM_OF_LINE_SENSORS; i++)
    {
        max = max_line_sensors_valu_[i];
        min = min_line_sensors_valu_[i];

        if(pre_max[i] != max)      result = false;
        else if(pre_min[i] != min) result = false;

        pre_max[i] = max;
        pre_min[i] = min;
    }

    if(result)
    {
        if(timer < CALIBRATION_SUCCESS_TIME)
        {
            timer++;
            result = false;
        }
    }
    else timer = 0;
    
    return result;
}

#ifdef DEBUG_MODE
void LineSensor::MonitorArrays()
{
    for(uint8_t i = 0; i < NUM_OF_LINE_SENSORS; i++)
    {
        g_line_buff[i] = line_sensors_buff_[i];
    }
}
#endif // DEBUG_MODE