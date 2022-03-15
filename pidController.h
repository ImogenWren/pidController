


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


  ## The Maths
  #### P = current_error;

     i.e current_error = setpoint - sensor_reading

  Example: setpoint = 100, sensor_reading = 120

          -20 = 100 -120

   therefore P = -20


  ####  I = (I + current_error) * dt;

  iteration 1, I = 0

  I = (0 + -20) * dt
  I = -20


  #### D = (current_error - previous_error) / dt;

  D = -20 - (-10) / dt

  This doesn't actually tell us much, but I think it was worth something.

  ## Full Algorithm:

  P = setpoint - sensor_reading
  I = (I + P) * dt
  D = (P - previous_P) / dt

  PID_error_correction = P*Kp + I*Ki + D*Kd

  NEW_OUTPUT = old_output + PID_error_correction

*/




#ifndef pidController_h
#define pidController_h

#if (ARDUINO >=100)
#include <Arduino.h>
#else
#include <wProgram.h>
#endif


#include "globals.h"
#include "dataObject.h"   // Used for input & output filtering
#include <autoDelay.h>



// Hardware

#define ADC_PIN A0                      // sensor
#define SETPOINT_PIN A7                 // User Control
#define OUTPUT_PIN 9                    // Output
#define INDICATOR_PIN 3                 // User Indication
#define ANALOG_TEST A3                  // User Test Input/Control

// Variables/Settings

// Neither of these are used yet
//#define HISTORIC_SAMPLES 100
//int16_t error_history[HISTORIC_SAMPLES];
#define MAX_DEFLECTION 50  // swing changes in output limited by this amount

#define SELF_CALIBRATION false


#define SAMPLE_RATE 1000       // Sample rate for measured_value (Hz)
#define INPUT_SAMPLERATE 1000      // sample rate for User inputs (Hz)
#define OUTPUT_UPDATE 1000      // Rate for output updates (Hz)
#define PRINT_RATE 1000           // Rate serial print data is printed (Hz)

#define DEADBAND 0        // dead band value for hysterisis.




#define SENSOR_MIN 100   // Measured for this specific sensor, ATM onlu used to LIMIT the SETPOINT to within a range the sensor can actually accomplish
#define SENSOR_MAX 800   //
// Measured sensor Min = 22
// Measured sensor Max = 843

#define OUTPUT_MIN 0       // OR use variables for self calibration
#define OUTPUT_MAX 255

//#define SELF_CALIBRATION true



//sensorMinMax sensorCal;  // global

#define IN_FILTER_BIAS 0.01     // Lots of filtering on input makes it smooth and easy for the PID controller to work  // 0 to 1: Higher numbers = faster response less filtering // Lower numbers = Slower response, more filtering
//#define OUT_FILTER_BIAS 0.7
#define OUT_FILTER_BIAS 0.7

// Low to no filtering on output makes it extremly fas tto react, however this wouldnt work as easily on a physical system with inertia or structural limitations etc.


// Just a thought. Instead of adding all of the sensor reads to this library, it should be maths only?


class pidController {

  public:

    // Constructor
    pidController();


    void begin();


    uint32_t sample_delay_uS;
    uint32_t output_delay_uS;
    uint32_t print_delay_mS;
    uint32_t input_delay_mS;



    sensorMinMax sensorCal;

    int16_t sensor_value;
    int16_t setpoint;

    int16_t output_value;

    uint8_t last_output_value = 0;   // This one is constrained because it only ever holds the constrained value

    int16_t current_error = 0;
    int16_t previous_error = 0;


    int16_t average_error = 0;    // Past N samples NOT IMPLEMENTED YET

    int16_t output_swing;  // Not implemented yet

    //uint32_t dt = 1;             // loop interval time - seconds?
    float dt = 1.0;                 // dt  = Loop interval time. dt = 1/SAMPLE_RATE


    float P = 0;
    float I = 0;
    float D = 0;


    float Kp = 0.6;     // If we start thinking of these values logically. P = total error, therefore Kp = 1 means the entire error value between 2 samples, will be "corrected" for in a single clock cycle. Lower numbers decrease overall filter responsiveness
    float Ki = 0.01;
    float Kd = 0.3;








    int16_t smoothInput(int16_t sensor_value);

    int16_t smoothOutput(int16_t output_value);


    int16_t PIDcontroller(int16_t setpoint, int16_t sensor_value, int16_t current_output);




    float PIDgain(float P, float I, float D, float Kp, float Ki, float Kd);



    int16_t averageError(int16_t latest_error);  // Calculate the average error over the last N samples





    void printOutput();

    void plotOutput();

    void plotHeader();






    uint16_t generateTest(uint16_t low_map, uint16_t high_map);


    struct sensorMinMax sensorSelfCalibrate();


    // Constants




    dataObject inputFilter(float filter_bias = IN_FILTER_BIAS);
    dataObject outputFilter(float filter_bias = OUT_FILTER_BIAS);

    uint32_t calculateSampleDelay(uint32_t sample_rate);
    uint32_t calculateOutputDelay(uint32_t sample_rate) ;
    uint32_t calculatePrintDelay(uint32_t print_rate) ;
    uint32_t calculateInputDelay(uint32_t input_rate);

    autoDelay sampleDelay;
    autoDelay printDelay;
    autoDelay inputDelay;
    autoDelay outputDelay;



  private:






};




#endif


























/*

  ## How to Tune PID Controller Manually

  _This means nothing to me, what is reset time?_
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
