/*    # PID Controller

        _Implementing a bare bones PID Controller from the ground up_

        **Imogen Wren**

        27/02/2022

  ## What is a PID Controller?

        A proportional feedback controller that aims to reduce the error between a measured value and a target value

  _What is PID?_

  ### ___P___ Proportional tuning:
  _involves correcting a target proportional to the difference.
  Thus, the target value is never achieved because as the difference approaches zero,
  so too does the applied correction._

  ### ___I___ Integral tuning:
  _attempts to remedy this by effectively cumulating the
  error result from the "P" action to increase the correction factor.
  For example, if the oven remained below temperature,
  “I” would act to increase the head delivered.
  However, rather than stop heating when the target is reached,
  "I" attempts to drive the cumulative error to zero, resulting in an overshoot._

  ### ___D___ Derivative tuning:
  _attempts to minimize this overshoot by slowing the correction factor applied as the target is approached._
*/





#include "pidController.h"



void pidController::begin() {
  Serial.begin(115200);
  //  pinMode(OUTPUT_PIN, OUTPUT);
  //  pinMode(INDICATOR_PIN, OUTPUT);
  sample_delay_uS = pidController::calculateSampleDelay(SAMPLE_RATE);
  // Serial.print("sample_delay_uS = ");
  // Serial.println(sample_delay_uS, 10);
  // dt = 1/float(sample_delay_uS);
  // dt = 1/float(SAMPLE_RATE);
  // dt = SAMPLE_RATE;
  //  Serial.print("dT = ");
  //  Serial.println(dt, 7);
  // delay(2000);
  output_delay_uS = pidController::calculateOutputDelay(OUTPUT_UPDATE);
  print_delay_mS = pidController::calculatePrintDelay(PRINT_RATE);
  input_delay_mS = pidController::calculateInputDelay(INPUT_SAMPLERATE);
  //  sensorCal = sensorSelfCalibrate();
  // Serial.print(sensorCal.Smin);
  // Serial.print("  :  ");
  // Serial.println(sensorCal.Smax);
  pidController::plotHeader();
}



int16_t pidController::smoothInput(int16_t sensor_value) {
  sensor_value = inputFilter.recursiveFilter(sensor_value)
                 return sensor_value;
}

int16_t pidController::smoothOutput(int16_t output_value) {
  output_value = outputFilter.recursiveFilter(output_value);
  return output_value;
}


int16_t pidController::PIDcontroller(int16_t setpoint, int16_t sensor_value, int16_t current_output) {
  // PID Functions
  current_error = setpoint - sensor_value;    // Current error is = proportional
  // Calculate seperate P, I & D errors (Do not confuse with P&ID!)
  P = current_error;
  I = (I + current_error) * dt;
  D = (current_error - previous_error) / dt;

// This function should really be split here
  
  float PID_correction = pidController::PIDgain(P, I, D, Kp, Ki, Kd);    // Each error measurement is multiplied by the gain for each channel then added.

  // Round Output Value to an int for output
  if (P >= DEADBAND || P <= DEADBAND) {
    output_value = output_value + int(PID_correction + 0.5);        //Origional Line
    output_swing = output_value - last_output_value;                // Not used currently, but might be useful to limit max swing per sample?
    output_value = constrain(output_value, 0 , 255);
    output_value = pidController::smoothOutput(output_value);
  }

  last_output_value = output_value;
  previous_error = current_error;
  return output_value;
}




float pidController::PIDgain(float P, float I, float D, float Kp, float Ki, float Kd) {
  float pid = (P * Kp) + (I * Ki) + (D * Kd);
  // Serial.println(pid);
  // Next Line Added
  return pid;
}



int16_t pidController::averageError(int16_t latest_error) {   // Calculate the average error over the last N samples
  //MATHS GO HERE
  return latest_error;
}


void pidController::printOutput() {
  char buffer[64];
  sprintf(buffer, "setpoint: [%i], sensor: [%i], error_c:[%i], error_p[%i], out[%i] ", setpoint, sensor_value, current_error, previous_error, output_value );
  Serial.println(buffer);
}

void pidController::plotOutput() {
  char buffer[64];
  sprintf(buffer, "%i, %i, %i, %i", setpoint, sensor_value, current_error, output_value);
  //  sprintf(buffer, "%i, %i, %i, %i, %i", setpoint, sensor_value, P, I, D);
  //sprintf(buffer, "%i, %i, %i, %i, %i", setpoint, sensor_value, current_error, previous_error, output_value );
  Serial.println(buffer);

}

void pidController::plotHeader() {
  char buffer[64];
  sprintf(buffer, "setpoint, sensor_value, current_error, output_value");
  //sprintf(buffer, "setpoint, sensor_value, P, I, D");
  //sprintf(buffer, "%i, %i, %i, %i, %i", setpoint, sensor_value, current_error, previous_error, output_value );
  Serial.println(buffer);
}





uint16_t pidController::generateTest(uint16_t low_map, uint16_t high_map) {
  uint16_t test_value = analogRead(ANALOG_TEST);
  test_value = map(test_value, 0, 1024, low_map, high_map);
  return test_value;
}

#define MIN_BUFFER 20   // Lower range headroom for target sensor value
#define MAX_BUFFER 120   // upper range headroom for target sensor value

uint16_t pidController::generateSetpoint() {
  uint16_t setpoint = analogRead(SETPOINT_PIN);
  setpoint = map(setpoint, 0, 1024, sensorCal.Smin + MIN_BUFFER, sensorCal.Smax - MAX_BUFFER);
  return setpoint;
}








uint32_t pidController::calculateSampleDelay(uint32_t sample_rate) {
  uint32_t sample_delay_uS = 1000000 / sample_rate;
  return sample_delay_uS;
}

uint32_t pidController::calculateOutputDelay(uint32_t sample_rate) {
  uint32_t output_delay_uS = 1000000 / sample_rate;
  return output_delay_uS;
}

uint32_t pidController::calculatePrintDelay(uint32_t print_rate) {
  uint32_t print_delay_mS = 1000 / print_rate;
  return print_delay_mS;
}


uint32_t pidController::calculateInputDelay(uint32_t input_rate) {
  uint32_t input_delay_mS = 1000 / input_rate;
  return input_delay_mS;
}
