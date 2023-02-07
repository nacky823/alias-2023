#include "imu.hpp"

void Imu::DegreeStackZ()
{
    double raw_z = iim_42652.GyroZLeft() + iim_42652.GyroZRight() / 2.0;
    double degree_z = raw_z * NORMAL_FUCOR_DPS * TIM6_PERIOD;

    degree_stack_z_ += degree_z;
}

void Imu::ResetDegreeStackZ()
{
    degree_stack_z_ = 0.0;
}

double Imu::GetDegreeStackZ()
{
    return degree_stack_z_;
}