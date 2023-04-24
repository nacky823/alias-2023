#ifndef IMU_HPP_
#define IMU_HPP_

#include "iim_42652.hpp"

#define RAD_LPF_RATIO  0.7
#define CONST_RAD_CALC 0.001065 // float

/* raw_imu_value (int16_t) : raw (-32768 <= raw <= 32767) [-]
 * normalization           : norm = 32768 [-]
 * sensitivity (+-2000dps) : sens = 2000 [deg/s]
 *
 * ans[rad] = raw[-] / norm[-] * sens[deg/s] / 180[deg] * pi[rad] * timer_period[s]
 *
 * constant[rad/s] = sens[deg/s] * pi[rad] / ( norm[-] * 180[deg] )
 *                 = 0.00106526443603169529841533860381 [rad/s]
 *
 * ans[rad] = raw[-] * constant[rad/s] * timer_period[s]
 */

class Imu
{
private:
    Iim42652 *iim_42652_;

    float rad_z_;
    float rad_stack_z_;
    bool clear_integral_error_;

public:
    Imu(Iim42652 *);
    void Update();
    void ClearRadStackZ();
    float GetRadStackZ();
    float PidControl(float);

};

#endif // IMU_HPP_