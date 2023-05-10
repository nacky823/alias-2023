#ifndef IMU_HPP_
#define IMU_HPP_

#include "iim_42652.hpp"

#define GYRO_P_GAIN 100
#define GYRO_I_GAIN 0
#define GYRO_D_GAIN 0
#define RAD_LPF_RATIO  0.7
#define CONST_RAD_CALC 0.001065 // float
#define NUM_OF_SAMPLE_CALIB 180000 // tim6[ms]
#define CALIB_OFFSET_R 0.0
#define CALIB_OFFSET_L 0.0

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

#ifdef DEBUG_MODE
    void CalibrationStackRadian();
#endif // DEBUG_MODE

public:
    Imu(Iim42652 *);
    void Update();
    void ClearRadStackZ();
    float GetRadStackZ();
    float GetRadZ();
    float PidControl(float);

#ifdef DEBUG_MODE
    bool Calibration();
#endif // DEBUG_MODE

};

#endif // IMU_HPP_