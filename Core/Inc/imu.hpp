#ifndef IMU_HPP_
#define IMU_HPP_

#include "iim_42652.hpp"

class Imu
{
private:

    Iim42652 *iim_42652_;

public:
    Imu(Iim42652 *);

};

#endif // IMU_HPP_