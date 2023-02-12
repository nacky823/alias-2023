#ifndef MACRO_H_
#define MACRO_H_

/* ----- Debug mode ----- */
#define DEBUG_MODE

/* ----- Parameter adjustment ----- */
#define INIT_DEBUG_MOTOR_DUTY 0.0
#define V_P_1 0.0
#define V_I_1 0.0
#define V_D_1 0.0
#define LINE_P_1 0.0
#define LINE_I_1 0.0
#define LINE_D_1 0.0
#define MIN_VELOCITY 1.3
#define COMMON_DUTY 0.2

/* ----- Velocity control ----- */
#define LPF_VELOCITY     0.70
#define VELOCITY_P_GAIN  0.0
#define VELOCITY_I_GAIN  0.0
#define VELOCITY_D_GAIN  0.0
#define LIMIT_TRANS_DUTY 0.90

/* ----- Line sensor ----- */
#define LINE_SENSOR_CORRECTION 0.950
#define NUMBER_OF_ADC 14
#define HALF_NUMBER_OF_ADC 7
#define NUMBER_OF_SAMPLE 10
#define INDEX_OF_SAMPLE_MEDIAN 5

/* ----- IMU ----- */
#define NORMAL_FUCOR_DPS 0.061037

/* ----- Internal timer ----- */
#define TIM6_PERIOD 1.25 // [ms]
#define TIM6_PERIOD_S 0.00125 // [s]

/* ----- Flash memory address ----- */
#define SECTOR_1_ADDRESS_HEAD 0x08004000 // 16 [KB]
#define SECTOR_1_ADDRESS_TAIL 0x08007FFF
#define SECTOR_2_ADDRESS_HEAD 0x08008000 // 16 [KB]
#define SECTOR_2_ADDRESS_TAIL 0x0800BFFF
#define SECTOR_3_ADDRESS_HEAD 0x0800C000 // 16 [KB]
#define SECTOR_3_ADDRESS_TAIL 0x0800FFFF
#define SECTOR_4_ADDRESS_HEAD 0x08010000 // 64 [KB]
#define SECTOR_4_ADDRESS_TAIL 0x0801FFFF

#endif // MACRO_H_