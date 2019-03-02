
#if !defined CONFIG_H
#define CONFIG_H

//Hardware configuration
#define LED_PIN 7
#define NUM_LEDS 60
#define IR_PIN 8

#define COLOR_MODE GRB//GRB

//----------- Configuration -------------
//LED configuration
#define VAL_MAX_DB 20
#define RGB_MAX 255
#define RGB_MIN 0
#define Rout 20
#define Gin 12
#define Gout 24
#define Bin 20
#define Bout 32
#define SMOOTHNESS 5
#define MAX_LED_LENGTH 5
#define MAX_BASS_LENGTH 20
#define MAX_VOICE_LENGTH 20
#define BACKGROUND_MODE_3 0
#define BACKGROUND_MODE_1 10

//FFT configuration
#define NSAMPLES 64
#define THRESHOLD 1000
#define SAMPLING_FREQUENCY 200000

//---------- Debugging ------------------
//#define DEBUG_TRACES_TIME
//#define DEBUG_TRACES_FREQ
//#define DEBUG_TRACES_FREQ_SPECTRUM
//#define DEBUG_TRACES_RGB
//#define DEBUG_TRACES_LED
//#define DEBUG_TRACES_IR

#endif
