#include "RGB_to_IR.h"


uint32_t RGB_to_IR::RGB2IRCode(uint8_t R, uint8_t G, uint8_t B)
{
  //Map RGB to IR Code
}

void RGB_to_IR::send_ir_command(uint32_t command, uint8_t repetitions)
{
  for(uint8_t i = 0; i < repetitions; i++)
  {
    irsend.sendNEC(command, 32);
    delay(SEND_PERIOD);
  }
}

RGB_to_IR::RGB_to_IR()
{
  irsend.begin();
}

void RGB_to_IR::send_ir_on()
{
  send_ir_command(0xFFE01F,NUM_IR_REPETITIONS);
}

void RGB_to_IR::send_ir_off()
{
  send_ir_command(0xFF609F,NUM_IR_REPETITIONS);
}

void RGB_to_IR::send_ir_rgb(uint8_t R, uint8_t G, uint8_t B)
{
  uint32_t command = RGB2IRCode(R,G,B);
  send_ir_command(command,NUM_IR_REPETITIONS);
}
