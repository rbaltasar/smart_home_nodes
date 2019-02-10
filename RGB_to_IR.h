#if !defined RGBTOIR_H
#define RGBTOIR_H

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <IRsend.h>
#include <IRutils.h>

#define NUM_IR_REPETITIONS 10
#define SEND_PERIOD 100 //milliseconds

#define NUM_IR_COLORS 15

class RGB_to_IR
{

private:
  
  IRsend m_irsend;
  HardwareSerial* m_hwPrint;

  const uint8_t red_list[NUM_IR_COLORS]    = {255, 255, 255, 255, 255, 255, 255, 120, 115, 90, 60, 10, 0, 0, 0};
  const uint8_t green_list[NUM_IR_COLORS]  = {255, 255, 170, 70, 60, 50, 0, 255, 0, 0, 255, 130, 255, 160, 0};
  const uint8_t blue_list[NUM_IR_COLORS]   = {255, 0, 20, 20, 20, 140, 0, 255, 90, 120, 130, 255, 0, 220, 255};
  const uint32_t codes_list[NUM_IR_COLORS] = {0xFFD02F, 0xFF8877, 0xFF9867, 0xFFA857, 0xFFB04F, 0xFF48B7, 0xFF906F, 0xFF28D7, 0xFF58A7, 0xFF6897, 0xFF30CF, 0xFF708F, 0xFF10EF, 0xFF18E7, 0xFF50AF};

  uint32_t RGB2IRCode(uint8_t R, uint8_t G, uint8_t B);
  void send_ir_command(uint32_t command, uint8_t repetitions);
  
  uint8_t get_indexes_closest(const uint8_t* const list_start,const uint8_t list_offset, const uint8_t list_size, uint8_t* indexes, const uint8_t value);
  uint32_t RGB2IRCode_map(uint8_t R, uint8_t G, uint8_t B);

  typedef struct {
    double r;       // a fraction between 0 and 1
    double g;       // a fraction between 0 and 1
    double b;       // a fraction between 0 and 1
  } rgb;

  typedef struct {
    double h;       // angle in degrees
    double s;       // a fraction between 0 and 1
    double v;       // a fraction between 0 and 1
  } hsv;

  rgb hsv2rgb(hsv in);

public:

  RGB_to_IR(uint8_t PIN_IR);
  void configure(HardwareSerial* hwPrint);
  void send_ir_on();
  void send_ir_off();
  void send_ir_rgb(uint8_t R, uint8_t G, uint8_t B);
  void send_ir_hsv(uint8_t H, uint8_t S, uint8_t V);
  void send_ir_brightness(bool brightness);
  void HSV_to_RGB(double H, double S, double V, uint8_t& R, uint8_t& G, uint8_t& B);

};

#endif
/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
