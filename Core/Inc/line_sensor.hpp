#ifndef LINE_SENSOR_HPP_
#define LINE_SENSOR_HPP_

#include "main.h"
#include "declare_extern.h"

#define LED_COMPARE           500
#define MAX_VALUE             1000
#define EMERGENCY_STOP_BORDER 700
#define CALIBRATION_COUNT     1500 // 1[ms]

class LineSensor
{
private:
    uint16_t adc_buffers_[NUMBER_OF_ADC];
    uint16_t consecutive_adc_buffers_[NUMBER_OF_SAMPLE][NUMBER_OF_ADC];
    uint16_t max_adc_values_[NUMBER_OF_ADC];
    uint16_t min_adc_values_[NUMBER_OF_ADC];
    uint16_t adc_values_[NUMBER_OF_ADC];
    uint8_t emergency_stop_flag_;

    void MergeSort(uint16_t [], uint8_t, uint8_t);

public:
    LineSensor();
    void Init();
    void StoreConsecutiveAdcBuffers();
    void UpdateAdcValues();
    float LeftRightDifference();
    uint8_t GetEmergencyStopFlag();
    uint8_t CheckCalibration();

#ifdef DEBUG_MODE
    void MonitorArrays();
#endif // DEBUG_MODE

};

#endif // LINE_SENSOR_HPP_