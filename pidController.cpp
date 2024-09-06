


#include "pidController.h"


pidController::pidController() {
}


void pidController::begin() {
}

void pidController::updateGain(float _kp, float _ki, float _kd) {
  Kp = _kp;  // Proportional Gain
  Ki = _ki;  // Integral Gain
  Kd = _kd;  // Derivative Gain
}

void pidController::updateLimits(int16_t minLimit, int16_t maxLimit){
  
  control_min = minLimit;
  control_max =  maxLimit;
}

// void updateSetpoint(int16_t new_setpoint);   // first specify setpoint

// void updateSensor(int16_t input_value);      // update library with current sensor value

int16_t pidController::PIDcontroller(float setpoint, float sensor_value, int16_t current_output) {
  current_sampleTime_mS = millis();
  dt_mS = current_sampleTime_mS - previous_sampleTime_mS;
  dt_S = float(dt_mS) / 1000.0;

  error_val = setpoint - sensor_value;


  P = float(error_val);
  I = (I + float(error_val)) * dt_S;
  D = (error_val - previous_error) / dt_S;

  float PID_correction = (P * Kp) + (I * Ki) + (D * Kd);

  control_val = constrain(current_output + int(round(PID_correction)), control_min, control_max);  // pid correction always rounded down why?

  previous_error = error_val;
  previous_sampleTime_mS = current_sampleTime_mS;
  return control_val;
}

 pidController::pidVals  pidController::return_pid_vals(){
  pidVals currentPID = {P, I, D};
  return currentPID;
 }