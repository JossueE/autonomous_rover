#ifndef ENCODER_H
#define ENCODER_H

#include <iostream>
#include <string>
#include <vector>
#include <math.h> 
#include <cstring>
#include <array>
#include <memory>

class ENCODER
{
public:
    void lefttick(float tick1);
    void righttick(float tick2);
    void reset_odometry();
    void computeOdom(double motors_axis, double TICKS_PER_METER, double current_time);

    double last_time = 0;
    float left_tic = 0, right_tic = 0;
    float last_left_ticks = 0, last_right_ticks = 0;

    //Robot states
    double x = 0.0;
    double y = 0.0;
    double th = 0.0;
    double xy_pos = 0.0;

    double vx = 0.0, raw_vx = 0.0; //double vy = 0.0;
    double vth = 0.0, raw_vth = 0.0;
    
    //First order low-pass filter vars (0 < alpha < 1)
    double fV_ant = 0.0, alpha_v = 0.95, fW_ant = 0.0, alpha_w = 0.99;
    
private:
    
};

#endif
