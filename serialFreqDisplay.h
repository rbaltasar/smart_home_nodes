#ifndef _MYLIBRARY_H
#define _MYLIBRARY_H
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "config.h"

class SerialFreqDisplay
{

private:

  HardwareSerial* m_hwPrint;
  int m_threshold, m_fft_width;

public:
    SerialFreqDisplay(uint8_t threshold, uint8_t fft_width);
    void begin(HardwareSerial* hwPrint);
    void printFreq(double* frequencies, double max_level_freq_instant);
    void printVals(double* frequencies);
    void printVals_char(char* frequencies);
};

#endif

/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
