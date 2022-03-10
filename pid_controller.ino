
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
#include <dataObject.h>

// Hardware

#define ADC_PIN A0                      // sensor
#define SETPOINT_PIN A7                 // User Control
#define OUTPUT_PIN 9                    // Output
#define INDICATOR_PIN 3                 // User Indication
#define ANALOG_TEST A3                  // User Test Input/Control

// Variables/Settings


#define SAMPLE_RATE 1000       // Sample rate for measured_value (Hz)
#define INPUT_SAMPLERATE 1000      // sample rate for User inputs (Hz)
#define OUTPUT_UPDATE 1000      // Rate for output updates (Hz)
#define PRINT_RATE 1000           // Rate serial print data is printed (Hz)

#define DEADBAND 5        // dead band value for hysterisis.

autoDelay sampleDelay;
autoDelay printDelay;
autoDelay inputDelay;
autoDelay outputDelay;

uint32_t sample_delay_uS;
uint32_t output_delay_uS;
uint32_t print_delay_mS;
uint32_t input_delay_mS;



//uint32_t dt = 1;             // loop interval time - seconds?
float dt = 1.0;                 // dt  = Loop interval time. dt = 1/SAMPLE_RATE

void setup() {


  Serial.begin(115200);
  pinMode(OUTPUT_PIN, OUTPUT);
  pinMode(INDICATOR_PIN, OUTPUT);
  sample_delay_uS = calculateSampleDelay(SAMPLE_RATE);
  Serial.print("sample_delay_uS = ");
  Serial.println(sample_delay_uS, 10);
  // dt = 1/float(sample_delay_uS);
  // dt = 1/float(SAMPLE_RATE);
  // dt = SAMPLE_RATE;
  Serial.print("dT = ");
  Serial.println(dt, 7);
  delay(2000);
  output_delay_uS = calculateOutputDelay(OUTPUT_UPDATE);
  print_delay_mS = calculatePrintDelay(PRINT_RATE);
  input_delay_mS = calculateInputDelay(INPUT_SAMPLERATE);
  plotHeader();
}



int16_t output_value = 0;    // although output is contrained, maths shouldnt be
uint8_t last_output_value = 0;   // This one is constrained because it only ever holds the constrained value

int16_t current_error = 0;
int16_t previous_error = 0;
int16_t average_error = 0;    // Past N samples



// Neither of these are used yet
#define HISTORIC_SAMPLES 100
int8_t error_history[HISTORIC_SAMPLES];
#define MAX_DEFLECTION 50  // swing changes in output limited by this amount




#define SENSOR_MIN 120   // Measured for this specific sensor, ATM onlu used to LIMIT the SETPOINT to within a range the sensor can actually accomplish
#define SENSOR_MAX 800




#define IN_FILTER_BIAS 0.01     // Lots of filtering on input makes it smooth and easy for the PID controller to work  // 0 to 1: Higher numbers = faster response less filtering // Lower numbers = Slower response, more filtering
#define OUT_FILTER_BIAS 0.6    // Low to no filtering on output makes it extremly fas tto react, however this wouldnt work as easily on a physical system with inertia etc.

dataObject inputFilter(IN_FILTER_BIAS, false);
dataObject outputFilter(OUT_FILTER_BIAS, false);


float P = 0;
float I = 0;
float D = 0;


float Kp = 0.3;
float Ki = 0.001;
float Kd = 0.8;

/*
    Now by ADDING the PID value i.e. THE ADJUSTED AND WEIGHTED ERROR to the CURRENT OUTPUT VALUE. IT WORKS VERY WELL!!!

    Observations. Instead of fading gently, its ramping to min/max very quickly. i.e: Its doing PWM on its own!

    However, this is producing a very smooth sensor reading and very quick tracking.

    I think partly this may be due to the filter? so I am going to experiment with them.

    also first test done with 99.8% proportional



   |       | Kp    | Ki    | Kd    |filterIn |filterOut  |   dT  |       Notes                                                                                    |
   |---    |---    |---    |---    |---      |---        |---    |---                                                                                             |
   | Test 1|0.99   | 0.0   |  0.01 |   0.01  |  0.01     |  1    |  Output ramped up and down very quickly but smooth sensor reading. samples @ input1000 output 10000|
   | Test 2|0.99   | 0.0   |  0.01 |   0.01  |  1.0      |  1    |  Output ramped up and down more quickly but smooth sensor reading. samples @ input1000 output 10000|
   | Test 3|0.99   | 0.0   |  0.01 |   0.01  |  0.01     |  1    |  Output ramped up and down very quickly but smooth sensor reading. samples @ input1000 output 10000|

  PS realised that the waverform was not printing the filter smoothed output, this has been fixed

*/

uint16_t  sensor_value;
uint16_t  setpoint;


int32_t output_swing;




void loop() {

  // Debugging/Monitoring Output
  if (printDelay.millisDelay(print_delay_mS)) {
    //printOutput();
    plotOutput();
  }



  // Input Functions

  if (sampleDelay.microsDelay(sample_delay_uS)) {
    sensor_value = readSensor();
    sensor_value = inputFilter.recursiveFilter(sensor_value);
  }


  if (inputDelay.millisDelay(input_delay_mS)) {
    setpoint = generateSetpoint();
    // setpoint = dataLib.recursiveFilter(setpoint);
  }



  output_value = PIDcontroller(setpoint, sensor_value, output_value);


  // Output Functions

  // Physical Output
  if (outputDelay.microsDelay(output_delay_uS)) {
    updateOutput(output_value);
  }
  // Test output to gather data on low and high range of LDR sensor
  // updateOutput(generateTest(0, 255));
}









int16_t PIDcontroller(int16_t setpoint, int16_t sensor_value, int16_t current_output) {
  // PID Functions
  current_error = setpoint - sensor_value;    // Current error is = proportional
  // Calculate seperate P, I & D errors (Do not confuse with P&ID!)
  P = current_error;
  I = (I + current_error) * dt;
  D = (current_error - previous_error) / dt;
  float PID_correction = PIDgain(P, I, D, Kp, Ki, Kd);    // Each error measurement is multiplied by the gain for each channel then added.

  // Round Output Value to an int for output
  if (P >= DEADBAND || P <= DEADBAND) {
    output_value = output_value + int(PID_correction + 0.5);//Origional Line
    output_swing = output_value - last_output_value;    // Not used currently, but might be useful to limit max swing per sample?
    output_value = constrain(output_value, 0 , 255);
    output_value = outputFilter.recursiveFilter(output_value);
  }

  last_output_value = output_value;
  previous_error = current_error;

  return output_value;

}




float PIDgain(float P, float I, float D, float Kp, float Ki, float Kd) {
  float pid = (P * Kp) + (I * Ki) + (D * Kd);
  // Serial.println(pid);
  // Next Line Added
  return pid;
}



int16_t averageError(int16_t latest_error) {   // Calculate the average error over the last N samples
  //MATHS GO HERE
  return latest_error;
}


void printOutput() {
  char buffer[64];
  sprintf(buffer, "setpoint: [%i], sensor: [%i], error_c:[%i], error_p[%i], out[%i] ", setpoint, sensor_value, current_error, previous_error, output_value );
  Serial.println(buffer);
}

void plotOutput() {
  char buffer[64];
  sprintf(buffer, "%i, %i, %i, %i", setpoint, sensor_value, current_error, output_value);
  //sprintf(buffer, "%i, %i, %i, %i, %i", setpoint, sensor_value, current_error, previous_error, output_value );
  Serial.println(buffer);

}

void plotHeader() {
  char buffer[64];
  sprintf(buffer, "setpoint, sensor_value, current_error, output_value");
  //sprintf(buffer, "%i, %i, %i, %i, %i", setpoint, sensor_value, current_error, previous_error, output_value );
  Serial.println(buffer);
}

void updateOutput(uint8_t output_value) {
  analogWrite(OUTPUT_PIN, output_value);
  analogWrite(INDICATOR_PIN, output_value);    // Mirror output for easy indication
}





uint16_t generateTest(uint16_t low_map, uint16_t high_map) {
  uint16_t test_value = analogRead(ANALOG_TEST);
  test_value = map(test_value, 0, 1024, low_map, high_map);
  return test_value;
}


uint16_t generateSetpoint() {
  uint16_t setpoint = analogRead(SETPOINT_PIN);
  setpoint = map(setpoint, 0, 1024, SENSOR_MIN, SENSOR_MAX);
  return setpoint;
}

uint16_t readSensor() {
  sensor_value = analogRead(ADC_PIN);
  return sensor_value;
}


uint32_t calculateSampleDelay(uint32_t sample_rate) {
  uint32_t sample_delay_uS = 1000000 / sample_rate;
  return sample_delay_uS;
}

uint32_t calculateOutputDelay(uint32_t sample_rate) {
  uint32_t output_delay_uS = 1000000 / sample_rate;
  return output_delay_uS;
}

uint32_t calculatePrintDelay(uint32_t print_rate) {
  uint32_t print_delay_mS = 1000 / print_rate;
  return print_delay_mS;
}


uint32_t calculateInputDelay(uint32_t input_rate) {
  uint32_t input_delay_mS = 1000 / input_rate;
  return input_delay_mS;
}
