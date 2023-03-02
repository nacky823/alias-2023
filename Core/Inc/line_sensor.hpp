#ifndef LINE_SENSOR_HPP_
#define LINE_SENSOR_HPP_

#include "main.h"
#include "declare_extern.h"

#define LINE_SENSORS_LED_COMPARE 500  // PWM COMPARE
#define HALF_NUM_OF_LINE_SENSORS 7    // Use left-right difference.
#define HALF_CONSECUTIVE_TIMES   5    // Use sorted median.
#define MAX_NORMALIZED_VALU      2000 // Normalization
#define EMERGENCY_STOP_BORDER    1700 // Normalization
#define CALIBRATION_SUCCESS_TIME 1500 // tim6[ms]

#ifdef DEBUG_MODE
extern bool g_error_handler_adc1;
extern bool g_error_handler_tim11;
extern uint16_t g_line_buff[NUM_OF_LINE_SENSORS];
extern uint16_t g_consecutive_line_buff[NUM_OF_LINE_SENSORS][CONSECUTIVE_TIMES];
extern uint16_t g_max_line_valu[NUM_OF_LINE_SENSORS];
extern uint16_t g_min_line_valu[NUM_OF_LINE_SENSORS];
extern uint16_t g_line_valu[NUM_OF_LINE_SENSORS];
extern uint32_t g_line_left;
extern uint32_t g_line_right;
#endif // DEBUG_MODE

class LineSensor
{
private:
    uint16_t line_sensors_buff_[NUM_OF_LINE_SENSORS];
    uint16_t consecutive_line_sensors_buff_[CONSECUTIVE_TIMES][NUM_OF_LINE_SENSORS];
    uint16_t min_line_sensors_valu_[NUM_OF_LINE_SENSORS];
    uint16_t max_line_sensors_valu_[NUM_OF_LINE_SENSORS];
    uint16_t line_sensors_valu_[NUM_OF_LINE_SENSORS];
    bool emergency_stop_flag_;

    void MergeSort(uint16_t [], uint8_t, uint8_t);

public:
    LineSensor();
    void Init();
    void StoreConsecutiveBuff();
    void New();
    void Update();
    bool GetEmergencyStopFlag();
    float LeftRightDifference();
    bool CheckCalibration();

#ifdef DEBUG_MODE
    void MonitorArrays();
#endif // DEBUG_MODE

};

#endif // LINE_SENSOR_HPP_