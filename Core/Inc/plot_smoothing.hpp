#ifndef PLOT_SMOOTHING_HPP_
#define PLOT_SMOOTHING_HPP_

#include "flash.hpp"


#define FLOAT_SIZE 4
#define INITIAL_INT_DATA 12345.6
#define MAX_LOG 3200

class PlotSmoothing
{
private:
    uint16_t now_address_;
    double radian_;
    double distance_;
    double x_coordinate_;
    double y_coordinate_;
    float X_[MAX_LOG];
    float y_[MAX_LOG];



    void StackRadian();
    void Smoothing(float *, uint16_t, uint16_t);

};

#endif // PLOT_SMOOTHING_HPP_