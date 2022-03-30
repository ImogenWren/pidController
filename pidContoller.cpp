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


//pidController::pidController(float input_filter_bias, float output_filter_bias) {
//}


// Sets up timings - later interrupts
void pidController::begin() {
  Serial.begin(115200);
  //  pinMode(OUTPUT_PIN, OUTPUT);
  //  pinMode(INDICATOR_PIN, OUTPUT);
  g_sample_delay_uS = pidController::calculateSampleDelay(SAMPLE_RATE);
  // Serial.print("sample_delay_uS = ");
  // Serial.println(sample_delay_uS, 10);
  // dt = 1/float(sample_delay_uS);
  // dt = 1/float(SAMPLE_RATE);
  // dt = SAMPLE_RATE;
  //  Serial.print("dT = ");
  //  Serial.println(dt, 7);
  // delay(2000);
  g_output_delay_uS = pidController::calculateOutputDelay(OUTPUT_UPDATE);
  g_print_delay_mS = pidController::calculatePrintDelay(PRINT_RATE);
  g_input_delay_mS = pidController::calculateInputDelay(INPUT_SAMPLERATE);
  //  sensorCal = sensorSelfCalibrate();
  // Serial.print(sensorCal.Smin);
  // Serial.print("  :  ");
  // Serial.println(sensorCal.Smax);
//  pidController::plotHeader();
}



void pidController::updateInput(int16_t input_value) {
  g_sensor_value = pidController::smoothInput(input_value);
}

void pidController::updateSetpoint(int16_t new_setpoint) {
  g_setpoint = new_setpoint;  
}


// THis can be private

int16_t pidController::smoothInput(int16_t sensor_value) {
  sensor_value = inputFilter.recursiveFilter((sensor_value));
  return sensor_value;
}

int16_t pidController::smoothOutput(int16_t output_value) {
  output_value = outputFilter.recursiveFilter(output_value);
  return output_value;
}

void pidController::updateGain(float P_gain, float I_gain, float D_gain) {
  g_Kp = P_gain;
  g_Ki = I_gain;
  g_Kd = D_gain;
}


int16_t pidController::PIDcontroller(int16_t setpoint, int16_t sensor_value, int16_t current_output) {
  // PID Functions
  g_current_error = setpoint - sensor_value;    // Current error is = proportional

  // Calculate seperate P, I & D errors (Do not confuse with P&ID!)
 // float P, I, D = 0;                                                                  // These variables DO need to be global - well at least the I term

  g_P = g_current_error;
  g_I = (g_I + g_current_error) * g_dt;
  g_D = (g_current_error - g_previous_error) / g_dt;

  // This function should really be split here

  float PID_correction = pidController::PIDgain(g_P, g_I, g_D, g_Kp, g_Ki, g_Kd);    // Each error measurement is multiplied by the gain for each channel then added.

  g_output_value = pidController::constrainOutput(PID_correction, DEADBAND, current_output);

  g_last_output_value = g_output_value;                    // Not sure this line is actually used in anything

  g_previous_error = g_current_error;         // previous error is used to calculate D term.

  return g_output_value;                     //Yes this needs a return, this library only does maths, final value must be passed to other functions, however software also has a global output_value
}




float pidController::PIDgain(float Pterm, float Iterm, float Dterm, float Pgain, float Igain, float Dgain) {
  float pid = (Pterm * Pgain) + (Iterm * Igain) + (Dterm * Dgain);
  // Serial.println(pid);
  // Next Line Added
  return pid;
}



int16_t pidController::constrainOutput(float pid_correction, int16_t deadband, int16_t current_output) {
  // Round Output Value to an int for output
  if (g_current_error >= deadband || g_current_error <= deadband) {
    current_output = current_output + int(pid_correction + 0.5);        //Origional Line round pid_correction tand cast to int
    current_output = constrain(current_output, 0 , 255);                   // Constrain to physical output limits - these limits should be adjustable via API!
    current_output = pidController::smoothOutput(current_output);
    g_output_swing = current_output - g_last_output_value;                // Not used currently, but might be useful to limit max swing per sample?
  }
  return current_output;
}


int16_t pidController::averageError(int16_t latest_error) {   // Calculate the average error over the last N samples
  //MATHS GO HERE
  return latest_error;
}


void pidController::printOutput() {
  char buffer[64];
  sprintf(buffer, "setpoint: [%i], sensor: [%i], error_c:[%i], error_p[%i], out[%i] ", g_setpoint, g_sensor_value, g_current_error, g_previous_error, g_output_value );
  Serial.println(buffer);
}

void pidController::plotOutput() {
  char buffer[64];
  sprintf(buffer, "%i, %i, %i, %i", g_setpoint, g_sensor_value, g_current_error, g_output_value);
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




sensorMinMax pidController::sensorSelfCalibrate() {
  sensorMinMax calibration;
  if (SELF_CALIBRATION) {
    //  updateOutput(OUTPUT_MIN);
    delay(1000); // allow input to stabalise
    // Averaging Script would be best
    // int16_t Smin = readSensor();
    //    updateOutput(OUTPUT_MAX);
    delay(1000);
    //   int16_t Smax = readSensor();
    //  calibration = {Smin, Smax};
  } else {
    calibration = {SENSOR_MIN, SENSOR_MAX};            // If not Self calibration, use manually provided values
  }
  return calibration;
}







uint32_t pidController::calculateSampleDelay(uint32_t sample_rate) {
  uint32_t sample_uS = 1000000 / sample_rate;
  return sample_uS;
}

uint32_t pidController::calculateOutputDelay(uint32_t sample_rate) {
  uint32_t output = 1000000 / sample_rate;
  return output;
}

uint32_t pidController::calculatePrintDelay(uint32_t print_rate) {
  uint32_t print_mS = 1000 / print_rate;
  return print_mS;
}


uint32_t pidController::calculateInputDelay(uint32_t input_rate) {
  uint32_t input_mS = 1000 / input_rate;
  return input_mS;
}
