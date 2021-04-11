// asservissement.cpp

#include "pid.h"


//===========
// PID CLASS
//===========
pid::pid(float kp, float ki, float kd): _kp(kp), _ki(ki), _kd(kd), _integralBoundary(1.0) {
  _integral = 0.0;
  _last_value = 0.0;
}


float pid::calculate(float target, float real, long dt) {
  _error = target - real;
  _derivative = 1000.0* _error  / dt;
  _integral += _error * dt / 1000.0;
  _integral = max(-_integralBoundary, min(_integralBoundary, _integral)); // boundary for integral
  _cmd = _kp * _error + _ki * _integral + _kd * _derivative; // calculate PID command
  return round(_cmd);
}



//=============
// RAMP CLASS
//=============
ramp::ramp(float a_max, float d_max, float v_max, float tolerance): _a_max(a_max), _d_max(d_max), _v_max(v_max), _tolerance(tolerance) {
  _v_previous = 0.0;
  _x_max = 1.0 * 0.5 * _v_max * _v_max / _d_max ;
}


float ramp::calculateSpeed(float target, float real, float current_speed, int dt) {
  _error = target - real;

  // calculate a target speed
  if (abs(_error) <= _tolerance) {
    _v = 0.0;
  }

  else if (_error > 0) {
    if (abs(_error) <= _x_max) {
      _v = -(_v_max / _x_max ) * (_x_max - _error - _tolerance) + _v_max;
    }
    else {
      _v = _v_max;
    }
  }
  else {
    if (abs(_error) <= _x_max) {
      _v = (_v_max / _x_max ) * (_x_max + _error - _tolerance) - _v_max;
    }
    else {
      _v = - _v_max;
    }
  }
  //_v_previous = current_speed;
  // Try to reach the target speed without being over the max acceleration
  if (_v > 0) {
    if (_v_previous > _v) {
      _v = max(_v, _v_previous - 0.000001 * _d_max * dt);
    }
    else {
      _v = min(_v, _v_previous + 0.000001 * _a_max * dt);
    }
  }
  else {
    if (_v_previous > _v) {
      _v = max(_v, _v_previous - 0.000001 * _a_max * dt);
    }
    else {
      _v = min(_v, _v_previous + 0.000001 * _d_max * dt);
    }
  }


  _v_previous = _v;
  return _v;
}
