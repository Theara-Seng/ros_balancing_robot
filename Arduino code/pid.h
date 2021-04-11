#ifndef PID_H
#define PID_H

#include "Arduino.h"



class pid{
  public:
    pid(float kp, float ki, float kd);
    float calculate(float target, float real, long dt);
  private:
    const float _kp;
    const float _ki;
    const float _kd;
    float _error;
    float _integral;
    float _derivative;
    float _last_value;
    const float _integralBoundary;
    float _cmd;
};


class ramp{
  public:
    ramp(float a_max, float d_max, float v_max, float tolerance);
    float calculateSpeed(float target, float real, float current_speed, int dt);
    const float _a_max;
  private:
    float _error;
    float _v;
    float _v_previous;
    float _x_max;
    float _d_max;
    const float _v_max;
    const float _tolerance;
};

#endif
