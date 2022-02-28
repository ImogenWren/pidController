
/*    PID Controller

        Implementing a bare bones PID Controller from the ground up

        Imogen Wren

        27/02/2022




*/

#include <autoDelay.h>

// Hardware

#define ADC_PIN A0
#define SETPOINT_PIN A7
#define OUTPUT_PIN 9

// Variables/Settings


#define SAMPLE_RATE 1000       // Sample rate for measured_value (Hz)
#define INPUT_SAMPLERATE 10      // sample rate for User inputs (Hz)
#define OUTPUT_UPDATE 10      // Rate for output updates (Hz)
#define PRINT_RATE 10           // Rate serial print data is printed (Hz)

#define DEADBAND 100           // dead band value for hysterisis.

autoDelay sampleDelay;
autoDelay printDelay;
autoDelay inputDelay;
autoDelay outputDelay;

uint32_t sample_delay_uS;
uint32_t output_delay_uS;
uint32_t print_delay_mS;
uint32_t input_delay_mS;

void setup() {
  Serial.begin(115200);
  pinMode(OUTPUT_PIN, OUTPUT);


  sample_delay_uS = calculateSampleDelay(SAMPLE_RATE);
  output_delay_uS = calculateOutputDelay(OUTPUT_UPDATE);
  print_delay_mS = calculatePrintDelay(PRINT_RATE);
  input_delay_mS = calculateInputDelay(INPUT_SAMPLERATE);



}

#define SENSOR_MIN 200
#define SENSOR_MAX 900

uint8_t output_value = 0;

int16_t current_error = 0;
int16_t previous_error = 0;



float P = 0;
float I = 0;
float D = 0;


float Kp = 10.0;
float Ki = 0;
float Kd = 0;

uint16_t  sensor_value;
uint16_t  setpoint;

uint32_t dt = 1;             // loop interval time - seconds?




void loop() {

  // Debugging/Monitoring Output
  if (printDelay.millisDelay(print_delay_mS)) {
    printOutput();
  }



  // Input Functions

  if (sampleDelay.microsDelay(sample_delay_uS)) {
    sensor_value = readSensor();
  }


  if (inputDelay.millisDelay(input_delay_mS)) {
    setpoint = generateSetpoint();
  }




  // PID Functions

  current_error = setpoint - sensor_value;    // Current error is = proportional

  P = current_error;

  I = I + current_error * dt;

  D = (current_error - previous_error) / dt;


  float PID_correction = PIDcontroller(P, I, D, Kp, Ki, Kd);

  // Round Output Value to an int for output

  output_value = int(PID_correction + 0.5);

  previous_error = current_error;


  output_value = constrain(output_value, 0 , 255);



  // Output Functions

  // Physical Output
  if (outputDelay.microsDelay(output_delay_uS)) {
    updateOutput(output_value);
    //  uint8_t test_output = map(setpoint, 0, 1023, 0 , 255);
    // updateOutput(test_output);
  }



}


float PIDcontroller(float P, float I, float D, float Kp, float Ki, float Kd) {

  float pid = (P * Kp) + (I * Ki) + (D * Kd);

  return pid;
}



void printOutput() {
  char buffer[64];
  sprintf(buffer, "setpoint: [%i], sensor: [%i], error_c:[%i], error_p[%i], out[%i] ", setpoint, sensor_value, current_error, previous_error, output_value );
  Serial.println(buffer);
}



void updateOutput(uint8_t output_value) {
  analogWrite(OUTPUT_PIN, output_value);
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
