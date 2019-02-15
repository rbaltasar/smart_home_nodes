/*

*/
#include "serialFreqDisplay.h"
#include "HardwareSerial.h"

SerialFreqDisplay::SerialFreqDisplay(uint8_t threshold,  uint8_t fft_width)
{

  m_threshold = threshold;
  m_fft_width = fft_width;

}

void SerialFreqDisplay::begin(HardwareSerial* hwPrint)
{
  m_hwPrint = hwPrint;
}

void SerialFreqDisplay::printFreq(double* frequencies, double max_level_freq_instant)
{

  for(uint8_t i = 0; i < m_fft_width - 1; i++)
  {
    if (frequencies[i] < (max_level_freq_instant * 0.9))
    {
      //m_hwPrint->print(frequencies[i]);
      m_hwPrint->print("-");
    }
    else
    {
      //m_hwPrint->print(frequencies[i]);
      //m_hwPrint->print(m_threshold);
      m_hwPrint->print("#");
    }
  }

  m_hwPrint->println();

}

void SerialFreqDisplay::printVals(double* frequencies)
{

  for(uint8_t i = 0; i < m_fft_width - 1; i++)
  {
    m_hwPrint->print(frequencies[i]);
    m_hwPrint->print(" ");
  }

  m_hwPrint->println();
}

void SerialFreqDisplay::printVals_char(char* frequencies)
{

  for(uint8_t i = 0; i < m_fft_width - 1; i++)
  {
    m_hwPrint->print((int)frequencies[i]);
    m_hwPrint->print(" ");
  }

  m_hwPrint->println();
}
