
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

dataObject average;

pidController PID;



autoDelay sampleDelay;
autoDelay printDelay;
autoDelay inputDelay;
autoDelay outputDelay;


#define P_GAIN 0.7
#define I_GAIN 0.01
#define D_GAIN 0.3

dataObject


void setup() {

  pinMode(OUTPUT_PIN, OUTPUT);
  pinMode(INDICATOR_PIN, OUTPUT);
  PID.begin();
  PID.updateGain(P_GAIN, I_GAIN, D_GAIN);
  // PID.sensorCal = PID.sensorSelfCalibrate(); < Some thought needs to go into this one, maybe calibrateSensorLOW(LOCAL_SENSOR_READING, LOW_BUFFER); and same for calSensorHIGH
  //  delay(2000);

  plotGenericHeader("Setpoint, Sensor, Error, Output, Average_Error");
}



int16_t output_value = 0;    // although output is contrained, maths shouldnt be





uint16_t  sensor_value;
uint16_t  setpoint;


int32_t output_swing;

int16_t error_value;
int16_t average_error;

// Variables

void loop() {





  // Input Functions

  if (sampleDelay.microsDelay(PID.g_sample_delay_uS)) {
    PID.updateInput(readSensor());
  }


  if (inputDelay.millisDelay(PID.g_input_delay_mS)) {
    PID.updateSetpoint(generateSetpoint());
  }


// Function here to calculate average error over the past N samples

 data_array[DATA_ARRAY_SIZE];    // Used for averaging methods





  PID.g_output_value = PID.PIDcontroller(PID.g_setpoint, PID.g_sensor_value, PID.g_output_value);
  error_value = PID.g_setpoint - PID.g_sensor_value;
  average.addDataPoint(error_value);
  average_error = average.calcMean();


  // Output Functions

  // Physical Output
  if (outputDelay.microsDelay(PID.g_output_delay_uS)) {
    PID.g_output_value = PID.PIDcontroller(PID.g_setpoint, PID.g_sensor_value, PID.g_output_value);
    error_value = PID.g_setpoint - PID.g_sensor_value;
    average.addDataPoint(error_value);
    average_error = average.calcMean();
    updateOutput(PID.g_output_value);
  }
  // Test output for sensor calibration
  //updateOutput(generateTest(0, 255));



  // Debugging/Monitoring Output
  if (printDelay.millisDelay(PID.g_print_delay_mS)) {
    //  PID.printOutput();
    // PID.plotOutput();
    plotGenericData(PID.g_setpoint, PID.g_sensor_value, PID.g_output_value, error_value, average_error);
  }
}



void plotGenericHeader(char header_string[46]) {
  char buffer[64];
  sprintf(buffer, "%s", header_string);
  Serial.println(buffer);
}

//<<<<<<< HEAD
// All hardware interactions done externally to library
//=======
void plotGenericData(int16_t dataZero, int16_t dataOne, int16_t dataTwo, int16_t dataThree, int16_t dataFour) {
  char buffer[64];
  sprintf(buffer, "%i, %i, %i, %i, %i, ", dataZero, dataOne, dataTwo, dataThree, dataFour);
  Serial.println(buffer);
}
//

int16_t readSensor() {
  sensor_value = analogRead(ADC_PIN);
  return sensor_value;
}


#define S_MIN  100
#define S_MAX  800


#define MIN_BUFFER 20   // Lower range headroom for target sensor value
#define MAX_BUFFER 120   // upper range headroom for target sensor value


int16_t generateSetpoint() {
  int16_t setpoint = analogRead(SETPOINT_PIN);
  // setpoint = map(setpoint, 0, 1024, PID.sensorCal.Smin + MIN_BUFFER, PID.sensorCal.Smax - MAX_BUFFER);  // Broken for now but can be fixed
  setpoint = map(setpoint, 0, 1024,  S_MIN, S_MAX);
  return setpoint;
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
