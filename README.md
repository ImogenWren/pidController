# PID Controller

_Implementing a bare bones PID Controller from the ground up_

  **Imogen Wren**
 
 10/03/2022

  ## What is a PID Controller?

  _A proportional feedback controller that aims to reduce the error between a measured value and a target value_

 ***What is PID?***

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
