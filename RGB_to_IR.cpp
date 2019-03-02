#include "RGB_to_IR.h"


RGB_to_IR::RGB_to_IR(uint8_t PIN_IR):
m_irsend(PIN_IR)
{
  m_irsend.begin();  
}

void RGB_to_IR::configure(HardwareSerial* hwPrint)
{
  m_hwPrint = hwPrint;
}

uint32_t RGB_to_IR::RGB2IRCode(uint8_t R, uint8_t G, uint8_t B)
{
  //Map RGB to IR Code
  uint32_t returnCode;
  
  if( (R == G) && (G == B) && (R == 255) )
  {
    returnCode = 0xFFD02F;
  }
  else if( R == 255 )
  {
    returnCode = 0xFF906F;
  }
  else if( G == 255 )
  {
    returnCode = 0xFF10EF;
  }
  else if( B == 255 )
  {
    returnCode = 0xFF50AF;
  }

   m_hwPrint->println(returnCode);

  return returnCode;
}

uint8_t RGB_to_IR::get_indexes_closest(const uint8_t* const list_start,const uint8_t list_offset, const uint8_t list_size, uint8_t* indexes, const uint8_t value_cmp)
{
  //Iterate through the list to find the closest value
  uint8_t closest_val = 0;
  uint8_t min_diff = 255;

  for(uint8_t i=list_offset; i < list_size+list_offset; i++)
  {
    uint8_t diff = abs(list_start[i] - value_cmp);

    if(diff < min_diff)
    {
      closest_val = list_start[i];
      min_diff = diff;
    }    
  }

  m_hwPrint->print("Closest val found: ");
  m_hwPrint->println(closest_val);

  uint8_t output_idx = 0;
  //Iterate through the list to find all the indexes containing the closest value
  for(uint8_t i=list_offset; i < list_size+list_offset; i++)
  {
    if(list_start[i] == closest_val)
    {
      m_hwPrint->print("Found index: ");
      m_hwPrint->println(i);
      indexes[output_idx] = i;
      output_idx++;
    }
  }

  return output_idx;
  
}

uint32_t RGB_to_IR::RGB2IRCode_map(uint8_t R, uint8_t G, uint8_t B)
{
  //Local memory to store the indexes
  uint8_t indexes[6];
  
  //Find the indexes of the closest values to R
  uint8_t num_indexes_R = get_indexes_closest(red_list, 0, NUM_IR_COLORS, indexes, R);

  //Finde the indexes of the closest values to G among the closest in R
  uint8_t num_indexes_G = get_indexes_closest(green_list, indexes[0] , num_indexes_R, indexes, G);

  //Finde the indexes of the closest values to B among the closest in R and G
  uint8_t num_indexes_B = get_indexes_closest(blue_list, indexes[0] , num_indexes_G, indexes, B);

  //Get the code of the found index
  uint8_t final_index = indexes[0];

  m_hwPrint->print("Final index: ");
  m_hwPrint->println(final_index);

  return codes_list[final_index];
  
}

RGB_to_IR::rgb RGB_to_IR::hsv2rgb(RGB_to_IR::hsv in)
{
  double      hh, p, q, t, ff;
  long        i;
  rgb         out;

    if(in.s <= 0.0) {       // < is bogus, just shuts up warnings
        out.r = in.v;
        out.g = in.v;
        out.b = in.v;
        return out;
    }
    hh = in.h;
    if(hh >= 360.0) hh = 0.0;
    hh /= 60.0;
    i = (long)hh;
    ff = hh - i;
    p = in.v * (1.0 - in.s);
    q = in.v * (1.0 - (in.s * ff));
    t = in.v * (1.0 - (in.s * (1.0 - ff)));

    switch(i) {
    case 0:
        out.r = in.v;
        out.g = t;
        out.b = p;
        break;
    case 1:
        out.r = q;
        out.g = in.v;
        out.b = p;
        break;
    case 2:
        out.r = p;
        out.g = in.v;
        out.b = t;
        break;

    case 3:
        out.r = p;
        out.g = q;
        out.b = in.v;
        break;
    case 4:
        out.r = t;
        out.g = p;
        out.b = in.v;
        break;
    case 5:
    default:
        out.r = in.v;
        out.g = p;
        out.b = q;
        break;
    }
    return out;     
}

void RGB_to_IR::send_ir_command(uint32_t command, uint8_t repetitions)
{
  digitalWrite(LED_BUILTIN, LOW);
  for(uint8_t i = 0; i < repetitions; i++)
  {
    m_irsend.sendNEC(command, 32);
    delay(SEND_PERIOD);
    yield();
  }
  digitalWrite(LED_BUILTIN, HIGH);
}

void RGB_to_IR::send_ir_on()
{
  send_ir_command(0xFFE01F,NUM_IR_REPETITIONS);
  m_hwPrint->println("Sending ON command");
}

void RGB_to_IR::send_ir_off()
{
  send_ir_command(0xFF609F,NUM_IR_REPETITIONS);
  m_hwPrint->println("Sending OFF command");
}

void RGB_to_IR::send_ir_rgb(uint8_t R, uint8_t G, uint8_t B)
{
   m_hwPrint->println("Sending RGB command");
   m_hwPrint->println(R);
   m_hwPrint->println(G);
   m_hwPrint->println(B);
   uint32_t command = RGB2IRCode_map(R,G,B);
   m_hwPrint->println("HEX IR command");
   m_hwPrint->println(command);
   send_ir_command(command,NUM_IR_REPETITIONS);
 
}

void RGB_to_IR::send_ir_hsv(uint8_t H, uint8_t S, uint8_t V)
{
  hsv val;
  val.h = H;
  val.s = S;
  val.v = V;

  m_hwPrint->println("Requested HSV command");
  m_hwPrint->println(H);
  m_hwPrint->println(S);
  m_hwPrint->println(V);

  rgb rgb_req = hsv2rgb(val);

  uint8_t R = (uint8_t)(255*rgb_req.r);
  uint8_t G = (uint8_t)(255*rgb_req.g);
  uint8_t B = (uint8_t)(255*rgb_req.b);


  m_hwPrint->println("Converted RGB command");
  m_hwPrint->println(R);
  m_hwPrint->println(G);
  m_hwPrint->println(B);

  send_ir_rgb(R,G,B);
  
}

void RGB_to_IR::HSV_to_RGB(double H, double S, double V, uint8_t& R, uint8_t& G, uint8_t& B)
{
  hsv val;
  val.h = H;
  val.s = S;
  val.v = V;

  m_hwPrint->println("Requested HSV command");
  m_hwPrint->println(H);
  m_hwPrint->println(S);
  m_hwPrint->println(V);

  rgb rgb_req = hsv2rgb(val);

  m_hwPrint->println("Converted RGB command float");
  m_hwPrint->println(rgb_req.r);
  m_hwPrint->println(rgb_req.g);
  m_hwPrint->println(rgb_req.b);

  R = (uint8_t)(255*rgb_req.r);
  G = (uint8_t)(255*rgb_req.g);
  B = (uint8_t)(255*rgb_req.b);


  m_hwPrint->println("Converted RGB command");
  m_hwPrint->println(R);
  m_hwPrint->println(G);
  m_hwPrint->println(B);
  
}


void RGB_to_IR::send_ir_brightness(bool brightness)
{
  if(brightness) send_ir_command(0xFFA05F,NUM_IR_REPETITIONS);
  else send_ir_command(0xFF20DF,NUM_IR_REPETITIONS);
  m_hwPrint->print("Sending brightness command: ");
  m_hwPrint->println(brightness);
}
