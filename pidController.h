/*    PID Controller

        Implementing a bare bones PID Controller from the ground up

        Imogen Wren

        27/02/2022

*/

#pragma once

#ifndef pidController_h
#define pidController_h

#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <wProgram.h>
#endif





class pidController {
  // Constructor

public:

  pidController();

  // global vars

//  int16_t setpoint_val;
 // int16_t sensor_val;
  float error_val;  // setpoint - current_val
  int16_t previous_error;

  int16_t control_val = 0;  // for bool o/p negative = off, positive = on

  int16_t control_min = -32768;
  int16_t control_max =  32767;

  uint32_t current_sampleTime_mS;
  uint32_t previous_sampleTime_mS = 0;

  uint32_t dt_mS = 1000;  // current_mS - last_mS
  float dt_S = 1.0;

  float P = 0;
  float I = 0;
  float D = 0;

  float Kp = 1.0;  // Proportional Gain
  float Ki = 0.0;  // Integral Gain
  float Kd = 0.0;  // Derivative Gain

  int16_t deadband; // define a range of values over which PID control is ignored and last state is maintained

  struct pidVals {
  float p;
  float i;
  float d;
};


  // Methods

  void begin();

  void updateGain(float _kp, float _ki, float _kd);

  void updateLimits(int16_t minLimit =-32768, int16_t maxLimit = 32767);

   // void updateSetpoint(int16_t new_setpoint);   // first specify setpoint

 // void updateSensor(int16_t input_value);      // update library with current sensor value

int16_t PIDcontroller(float setpoint, float sensor_value, int16_t current_output);

pidVals return_pid_vals();




private:
};

#endif