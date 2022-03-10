


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

#include <dataObject.h>   // Used for input & output filtering
#include <autoDelay.h>  




class pidController
{

  public:
    // Constructor

    pidController(float filterBias = 0.9, bool serialMonitor = false):
      printSerial(serialMonitor),
      w(filterBias)
    {
    }

    void begin(uint32_t baudrate = 115200);    // Serial Comms

    int32_t recursiveFilter(int32_t Xn);




    // Constants




    // Variables



  private:

    int32_t Ypre;   //Y(n-1) Variable used for recursive filter

    float w;             // bias value for recursive filter

    bool printSerial;

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
