#ifndef RUN_HPP_
#define RUN_HPP_

#include "stm32f4xx_hal.h"
#include "encoder.hpp"
#include "flash.hpp"
#include "iim_42652.hpp"
#include "imu.hpp"
#include "led.hpp"
#include "line_sensor.hpp"
#include "line_trace.hpp"
#include "logger.hpp"
#include "motor.hpp"
#include "print.hpp"
#include "rotary_switch.hpp"
#include "side_sensor.hpp"
#include "velocity_control.hpp"

/* run_mode_ */
#define RAM_LOG 0x0B
//#define GYRO_TEST   0x03
#define LINE_TRACE_DEBUG       0x02
#define VELOCITY_CONTROL_DEBUG 0x01
#define LINE_TRACE       0x02
#define VELOCITY_CONTROL 0x01

#define STANDBY          0x00
#define READY            0x0E
#define DEV              0x0C
#define DEV_ACCEL        0x0A
#define THIRD_RUN        0x08
#define FORTH_RUN        0x06
#define FIFTH_RUN        0x04

#define DEV_GOAL         0xCA
#define EMERGENCY        0xEE

#define FIRST_RUN        0x1F
#define FIRST_GOAL       0xF1
#define SECOND_RUN       0x2F
#define SECOND_GOAL      0xF2


#define SWITCH_INTERVAL_WAIT_TIME 200

class Run
{
private:
    void SetRunMode(uint8_t);
    void SetRunModeChangedInternal(bool);
    void WhenSwitchChange();
    bool SwitchChangeInterval(uint8_t);
    /* run_mode_ */
    void ModeEmergency();
    void ModeReady();
    void ModeStandby();
    //void ModeLog();
    void ModeGyroTest();
    void ModeDevelopment();
    void ModeDevAccel();
    void ModeDevGoal();
    void ModeFirstRun();
    void ModeFirstGoal();
    void ModeSecondRun();
    void ModeSecondGoal();
    void ModeVelocityControl();
    void ModeLineTrace();

    void ModeStopIntrrupt();
    /* Motor control */
    bool EmergencyStop();
    bool DevEmergencyStop();
    float DevTargetVelocity(uint8_t);
    float DevAccelTarget(uint8_t);
    float FirstTargetVelocity(uint8_t);
    float SecondTargetVelocity(uint8_t);
    float VelocityControlTarget(uint8_t);
    float CommonDuty(uint8_t);
    void UpdateRunMode(uint8_t);

#ifdef DEBUG_MODE
    void ModeVelocityControlDebug();
    void ModeLineTraceDebug();
    void ModeInitialDebug();
    void InitialTest();
#endif // DEBUG_MODE

    uint8_t run_mode_;
    bool run_mode_changed_internal_;
    uint8_t switch_interval_wait_timer_;
    bool switch_interval_wait_enable_;
    bool switch_interval_led_enable_;
    bool mode_complete_;
    bool store_log_failed_;

    Encoder *encoder_;
    Flash *flash_;
    Iim42652 *iim_42652_;
    Imu *imu_;
    Led *led_;
    LineSensor *line_sensor_;
    LineTrace *line_trace_;
    Logger *logger_;
    Motor *motor_;
    Print *print_;
    RotarySwitch *rotary_switch_;
    SideSensor *side_sensor_;
    VelocityControl *velocity_control_;

public:
    Run(Encoder *,
        Flash *,
        Iim42652 *,
        Imu *,
        Led *,
        LineSensor *,
        LineTrace *,
        Logger *,
        Motor *,
        Print *,
        RotarySwitch *,
        SideSensor *,
        VelocityControl *);
    void Init();
    void Timer7();
    void RunMode();
    void Timer2();

#ifdef DEBUG_FLASH
    void DubugFlash();
    uint8_t TestFlash();
#endif // DEBUG_FLASH

};

#endif // RUN_HPP_