#if !defined RGBTOIR_H
#define RGBTOIR_H

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <IRsend.h>

#define NUM_IR_REPETITIONS 2
#define SEND_PERIOD 500 //milliseconds

class RGB_to_IR
{

private:

  uint32_t RGB2IRCode(uint8_t R, uint8_t G, uint8_t B);
  void send_ir_command(uint32_t command, uint8_t repetitions)
  IRsend m_irsend(IR_LED);

public:

  RGB_to_IR();
  void send_ir_on();
  void send_ir_off();
  void send_ir_rgb(uint8_t R, uint8_t G, uint8_t B);

};

#endif
/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
