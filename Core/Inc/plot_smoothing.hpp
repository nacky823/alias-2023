#ifndef PLOT_SMOOTHING_HPP_
#define PLOT_SMOOTHING_HPP_

#include "flash.hpp"

#define FLOAT_SIZE 4
#define INITIAL_INT_DATA 12345.6
#define MAX_LOG 3200

class PlotSmoothing
{
private:
    double radian_;
    double distance_;
    double x_coordinate_;
    double y_coordinate_;
    float x_[MAX_LOG];
    float y_[MAX_LOG];

    Flash *flash_;

    void SetRadian(float);
    void StackRadian(uint16_t);
    void SetDistance(float);
    void StoreDistance(uint16_t);
    void CalculateCoordinate();
    void StoreCoordinate();
    void Smoothing(float *, uint16_t, uint16_t);
    void Print();

public:
    PlotSmoothing(Flash *);
    void Run();

};

#endif // PLOT_SMOOTHING_HPP_