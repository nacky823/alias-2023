#ifndef PLOT_SMOOTHING_HPP_
#define PLOT_SMOOTHING_HPP_

#include "flash.hpp"

class PlotSmoothing
{
private:
    float radian_stack_;

    void StackRadian();
    void Smoothing(float *, uint16_t, uint16_t);

};

#endif // PLOT_SMOOTHING_HPP_