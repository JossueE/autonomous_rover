#include "encoder.h"
#include <math.h>

void ENCODER::lefttick(float tick1){
    left_tic = tick1;
}

void ENCODER::righttick(float tick2){
    right_tic = tick2;
}

void ENCODER::reset_odometry(){
  x = 0;
  y = 0;
  th = 0;
}

void ENCODER::computeOdom(double motors_axis, double TICKS_PER_METER, double current_time)
{
    double dt = 0.001;
    float delta_L = 0, delta_R = 0;
    double dl = 0.0, dr = 0.0, dc = 0.0, dth = 0.0;

    delta_L = (left_tic - last_left_ticks);
    delta_R = (right_tic- last_right_ticks);
    
    //std::cout << "delta L,R: " << delta_L << ", " << delta_R << std::endl;
    
    //Correction when crossing through zero
    if( abs(delta_L) > 6000 ) {
      delta_L = 0;
      // std::cout << "delta_L = 0, Correction when crossing through zero\n";
    }
    if( abs(delta_R) > 6000 ) {
      delta_R = 0;
      // std::cout << "delta_R = 0, Correction when crossing through zero\n";
    }
    
    dl = ((double) delta_L) / TICKS_PER_METER;
    dr = ( (double) delta_R) / TICKS_PER_METER;
    
    dc = (dl + dr) / 2;
    dth = (dr-dl) / motors_axis;

    dt = current_time - last_time; //delta time in SECONDS

    /* Esta es la forma que se utilizó en la version para ROS 1, funciona igual pero es más compleja
    if (dr == dl){
        deltaX = dl*cos(th);
        deltaY = dl*sin(th);
    } else {
      radio = dc/dth;

      iccX=x-radio*sin(th);
      iccY=y+radio*cos(th);

      deltaX = cos(dth) * (x-iccX) - sin(dth) * (y-iccY) + iccX - x;
      deltaY = sin(dth) * (x-iccX) + cos(dth) * (y-iccY) + iccY - y;
    }

    x += deltaX;  
    y += deltaY;
    
    //th = wrapAngle( (double)(th+dth) );
    */
    
    //Another most common expression to compute the diff drive robot odometry
    x += dc*cos(th+dth/2);
    y += dc*sin(th+dth/2);
    th += dth;
    th = atan2( sin(th),cos(th) ); //Another simpler way to wrap the angle in [-pi,pi]


    if (dt > 0.0){
      raw_vx = dc / dt;
      raw_vth = dth / dt;

      vx = alpha_v*fV_ant + (1-alpha_v)*raw_vx; //filtered vx
      fV_ant = vx;

      vth = alpha_w*fW_ant + (1-alpha_w)*raw_vth; //filtered vth
      fW_ant = vth;
    }
    //vy = 0;

    xy_pos = sqrt(x*x + y*y);

    last_left_ticks = left_tic;
    last_right_ticks = right_tic;
    last_time = current_time;
}
