
/*    PID Controller

        Implementing a bare bones PID Controller from the ground up

        Imogen Wren

        27/02/2022

  Proportional tuning:
  involves correcting a target proportional to the difference.
  Thus, the target value is never achieved because as the difference approaches zero,
  so too does the applied correction.

  Integral tuning:
  attempts to remedy this by effectively cumulating the
  error result from the "P" action to increase the correction factor.
  For example, if the oven remained below temperature,
  “I” would act to increase the head delivered.
  However, rather than stop heating when the target is reached,
  "I" attempts to drive the cumulative error to zero, resulting in an overshoot.

  Derivative tuning:
  attempts to minimize this overshoot by slowing the correction factor applied as the target is approached.

  How to Tune PID Controller Manually
  Manual tuning of PID controller is done by:
  - setting the reset time to its maximum value and
  - rate to zero
  - increasing the gain until
  -> the loop oscillates at a constant amplitude.

  When the response to an error correction occurs quickly, a larger gain can be used.
  If response is slow a relatively small gain is desirable.

  Then:
  - set the gain to half of that value
  - adjust the reset time so it corrects for any offset within an acceptable period.

  - Finally, increase the rate until overshoot is minimized.


*/

#include <autoDelay.h>
#include "dataObject.h"
#include "pidController.h"



pidController PID();




void setup() {

  pinMode(OUTPUT_PIN, OUTPUT);
  pinMode(INDICATOR_PIN, OUTPUT);
  PID.begin();
  sensorMinMax = sensorSelfCalibrate();
  //  delay(2000);
}



int16_t output_value = 0;    // although output is contrained, maths shouldnt be





uint16_t  sensor_value;
uint16_t  setpoint;


int32_t output_swing;


autoDelay sampleDelay;
autoDelay printDelay;
autoDelay inputDelay;
autoDelay outputDelay;

// Variables
uint32_t sample_delay_uS;
uint32_t output_delay_uS;
uint32_t print_delay_mS;
uint32_t input_delay_mS;


void loop() {

  // Debugging/Monitoring Output
  if (printDelay.millisDelay(print_delay_mS)) {
    //printOutput();
    PID.plotOutput();
  }



  // Input Functions

  if (sampleDelay.microsDelay(sample_delay_uS)) {
    sensor_value = readSensor();
    sensor_value = PID.smoothInput(sensorValue);
  }


  if (inputDelay.millisDelay(input_delay_mS)) {
    setpoint = generateSetpoint();
    // setpoint = dataLib.recursiveFilter(setpoint);   // Set point doesnt need filtering for now
  }



  output_value = PID.PIDcontroller(setpoint, sensor_value, output_value);


  // Output Functions

  // Physical Output
  if (outputDelay.microsDelay(PID.output_delay_uS)) {
    updateOutput(output_value);
  }
  // Test output for sensor calibration
  //updateOutput(generateTest(0, 255));
}



PID.sensorMinMax sensorSelfCalibrate() {
  sensorMinMax calibration;
  if (SELF_CALIBRATION) {
    updateOutput(OUTPUT_MIN);
    delay(1000); // allow input to stabalise
    // Averaging Script would be best
    int16_t Smin = readSensor();
    updateOutput(OUTPUT_MAX);
    delay(1000);
    int16_t Smax = readSensor();
    calibration = {Smin, Smax};
  } else {
    calibration = {SENSOR_MIN, SENSOR_MAX};            // If not Self calibration, use manually provided values
  }
  return calibration;
}



uint16_t readSensor() {
  sensor_value = analogRead(ADC_PIN);
  return sensor_value;
}

void updateOutput(uint8_t output_value) {
  analogWrite(OUTPUT_PIN, output_value);
  analogWrite(INDICATOR_PIN, output_value);    // Mirror output for easy indication
}




/*


   |       | Kp    | Ki    | Kd    |filterIn |filterOut  |   dT  |       Notes                                                                                    |
   |---    |---    |---    |---    |---      |---        |---    |---                                                                                             |
   | Test 1|0.99   | 0.0   |  0.01 |   0.01  |  0.01     |  1    |  Output ramped up and down very quickly but smooth sensor reading. samples @ input1000 output 10000|
   | Test 2|0.99   | 0.0   |  0.01 |   0.01  |  1.0      |  1    |  Output ramped up and down more quickly but smooth sensor reading. samples @ input1000 output 10000|
   | Test 3|0.99   | 0.0   |  0.01 |   0.01  |  0.01     |  1    |  Output ramped up and down very quickly but smooth sensor reading. samples @ input1000 output 10000|

  PS realised that the waverform was not printing the filter smoothed output, this has been fixed

*/
