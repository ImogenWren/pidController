/* ~~~~~~~~~~~~~ dataObject.h ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

   Library to implement data sorting & filtering algorithms
      
   Data Filtering Algorithms:

   - Recursive Filter

   - Averaging Values


    Data Sorting:


    Started by

    Imogen Wren

    01.11.2020

*/



#ifndef dataObject_h
#define dataObject_h

#if (ARDUINO >=100)
#include <Arduino.h>
#else
#include <wProgram.h>
#endif

#define basicFilter recursiveFilter   // Synonyms for basic methods

#define DATA_ARRAY_SIZE 128

class dataObject
{

  public:
    // Constructor

    dataObject(float filterBias = 0.9, bool serialMonitor = false):
      printSerial(serialMonitor),
      w(filterBias)
    {
    }

    void begin(uint32_t baudrate = 115200);    // Serial Comms

    int32_t recursiveFilter(int32_t Xn);

 //   int16_t averageMode(int16_t *data_array);
    

    // Constants




    // Variables


 //   int16_t data_array[DATA_ARRAY_SIZE];    // Used for averaging methods

  private:

    int32_t Ypre;   //Y(n-1) Variable used for recursive filter

    float w;             // bias value for recursive filter

    bool printSerial;

};




#endif
