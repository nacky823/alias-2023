#include "run.hpp"

Run::Run()

void Run::EmergencyStop()
{
    static uint8_t count = 0;
    uint8_t line = line_sensor.GetEmergencyStopFlag();

    if(line == 1)
    {
        count++;
        if(count > EMERGENCY_SIDE_IGNORE) side_sensor.RaiseIgnoreFlag();
        if(count > EMERGENCY_STOP_COUNT) g_mode = RESET;
    }
    else count = 0;
}

void Run::FirstRun()
{
    float rotat = line_sensor.PidControl(LINE_KP_1, LINE_KI_1, LINE_KD_1);
    float trans = velocity_control.PidControl(TARGET_V_1, V_KP_1, V_KI_1, V_KD_1);

    motor.Drive(trans, rotat);
}

void GoalStop()
{
    uint8_t goal_count = side_sensor.GetGoalMarkerCount();

    if(side_sensor.GetGoalMarkerCount() >= 2) g_mode = FIRST_GOAL;
}