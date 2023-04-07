#ifndef MACRO_H_
#define MACRO_H_

/* ----- Mode selection ----- */
#define DEBUG_MODE
#define TEST_MODE

/* ----- Parameter setting ----- */
/* PID gain velocity control */
/* PID gain line trace "test" */

#define LINE_ONLY_P_GAIN 0.0001
#define LINE_ONLY_I_GAIN 0
#define LINE_ONLY_D_GAIN 0

#define INIT_DEBUG_MOTOR_DUTY 0.0
#define V_P_1 0.0
#define V_I_1 0.0
#define V_D_1 0.0
#define LINE_P_1 0.0
#define LINE_I_1 0.0
#define LINE_D_1 0.0
#define MIN_VELOCITY 0.5
#define COMMON_DUTY 0.2

#define EMERGENCY_STOP_TIME 5
#define EXPLORE_VELOCITY 0.60

#define SLOW_DRIVE_DUTY 0.1
#define SLOW_DRIVE_TIME 500
#define SLOW_DRIVE_VELOCITY 0.10
#define STOP_TIME 200

/* ----- Velocity control ----- */
#define VELOCITY_LPF_RATIO     0.70
#define VELOCITY_P_GAIN  1.0
#define VELOCITY_I_GAIN  0.0
#define VELOCITY_D_GAIN  0.0
#define LIMIT_TRANS_DUTY 0.90
#define MAX_VELOCITY     4.0

#define VELOCITY_CONTROL_TARGET 1.2

/* ----- Line sensor ----- */
#define LINE_SENSOR_CORRECTION 1.0
#define NUM_OF_LINE_SENSORS    14
#define CONSECUTIVE_TIMES      10

/* ----- IMU ----- */
#define NORMAL_FUCOR_DPS 0.061037

/* ----- Internal timer ----- */
#define TIM6_PERIOD   1.25    // [ms]
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

/* Address to store error code when flash write fails. */
#define ERROR_CODE_STORE_DISTANCE 0x08018000
#define ERROR_CODE_STORE_RADIAN   0x08018100
#define ERROR_CODE_STORE_VARIOUS  0x08018200
#define ERROR_CODE_STORE_ACCEL    0x08018300
#define ERROR_CODE_STORE_DECEL    0x08018400
#define STORE_CALIBRATION         0x08018500

#endif // MACRO_H_