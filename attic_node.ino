/*
  Home automation - attic node

  Data gathering:
    Temperature & humidity
    Light amount

  Data publishing:
    Temperature & humidity

  Topic subscription:
    attic/light

  Logic:

*/

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "RGB_to_IR.h"
#include <DHTesp.h>

/* Network settings */
const char* ssid = "";
const char* password = "";
const char* mqtt_server = "192.168.2.118";

/* Debugging */
#define DEBUG_ENABLED 1

/* Hardware settings */
#define TEMPERATURE_SENSOR_PIN 5
#define DHTTYPE DHT11   // DHT 11
#define LIGHT_SENSOR_PIN 2
#define IR_EMITTER_PIN 3
#define LIGHT_THRESHOLD 1
#define MEASUREMENT_DELTA 0.1
#define DEFAULT_POLLING_PERIOD 10000 //10 seconds
#define WARNING_POLLING_PERIOD 100000 //100 seconds

/* Data collection settings */
#define BUFFER_SIZE 50

/* Logical states */
float light_amount = 0;
float temperature_buffer[BUFFER_SIZE+1];
float humidity_buffer[BUFFER_SIZE+1];
float temperature = 0;
float temperature_old = 0;
float humidity = 0;
float humidity_old = 0;
uint8_t nan_count = 0;
uint32_t polling_period = DEFAULT_POLLING_PERIOD;

struct lamp_status
{
  String state;
  bool color;
  uint8_t R;
  uint8_t G;
  uint8_t B;
};

lamp_status lamp_request;

/* Communication settings */
WiFiClient espClient;
PubSubClient client(espClient);
long last_iteration = 0;
char msg[5];
//StaticJsonBuffer<200> jsonBuffer;
//JsonObject& JSONencoder = JSONbuffer.createObject();

DHTesp temperature_sensor;

void setup() {
  //pinMode(BUILTIN_LED, OUTPUT);     // Initialize the BUILTIN_LED pin as an output
  Serial.begin(115200);

  setup_wifi();
  setup_hardware();
  setup_mqtt();

  lamp_request.state = "";
  lamp_request.R = 0;
  lamp_request.G = 0;
  lamp_request.B = 0;

  temperature_buffer[BUFFER_SIZE] = 0;
  humidity_buffer[BUFFER_SIZE] = 0;
}

/* Subscribe to the MQTT topics */
void setup_mqtt()
{
  /* Define MQTT broker */
  client.setServer(mqtt_server, 1883);
  /* Define callback function */
  client.setCallback(callback);
  /* Subscribe to topics */
  client.subscribe("attic_node/light");
}

/* Connect to the wireless network */
void setup_wifi() {

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

/* Configure the hardware */
void setup_hardware()
{

 /* Setup pins */
 temperature_sensor.setup(TEMPERATURE_SENSOR_PIN, DHTesp::DHT11);

#if DEBUG_ENABLED
   Serial.println("Hardware setup");
#endif
}

/* Configure the callback function for a subscribed topic */
void callback(char* topic, byte* payload, unsigned int length) {

  /* Print message (debugging only) */
#if DEBUG_ENABLED
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
#endif

  /* Filter for topics (optional when >1 topics) */

  /* Read topic info */

  /* Parse JSON message */

  /* Save request in local memory */
  #if 0
  lamp_request.state =
  lamp_request.color =
  if(lamp_request.color)
  {
    lamp_request.R =
    lamp_request.G =
    lamp_request.B =
  }
  #endif
}

/* Reconnect to the MQTT broker */
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("AtticClient")) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      //client.publish("outTopic", "hello world");
      // ... and resubscribe
      client.subscribe("attic_node/light");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

/* Handle the network part of the loop */
void network_loop()
{
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
}

/* Get the amount of light */
void get_light_amount()
{
  light_amount = digitalRead(LIGHT_SENSOR_PIN);
#if 0
  Serial.print("Light amount: ");
  Serial.println(light_amount);
#endif
}

/* Average an array. Remove oldest element. Add newest element. Filter out NaN values */
float process_measurement(float* measurement_buffer, float measurement)
{
  float avg = 0;
  uint8_t measurement_counter = 0;

  //Get index of the oldest element, stored in position 0 of the buffer
  uint8_t oldest_idx = (uint8_t)(measurement_buffer[BUFFER_SIZE]);

  //Update index of the oldest element
  uint8_t next_idx;
  if( (oldest_idx+1) >= BUFFER_SIZE ) next_idx = 0;
  else next_idx = oldest_idx + 1;
  measurement_buffer[BUFFER_SIZE] = next_idx;

  //Replace oldest element by newest in the FIFO
  measurement_buffer[oldest_idx] = measurement;

  //Average the buffer skipping NaN values, unless ALL are NaNs
  for(uint8_t i=0; i<BUFFER_SIZE; i++)
  {
    float val = measurement_buffer[i];
    if(!isnan(val) && (val != 0))
    {
      avg += val;
      measurement_counter++;
    }
  }

  if(measurement_counter == 0) avg = NAN;
  else avg /= measurement_counter;

  //Return averaged value
#if DEBUG_ENABLED
  Serial.print("Averaged value: ");
  Serial.println(avg);
  Serial.print("Filtered values: ");
  Serial.println(measurement_counter);
#endif
  return avg;
}

/* Get temperature and humidity */
void get_temperature()
{
  float tmp_humidity = temperature_sensor.getHumidity();
  float tmp_temperature = temperature_sensor.getTemperature();

  temperature_old = temperature;
  humidity_old = humidity;

  temperature = process_measurement(temperature_buffer,tmp_temperature);
  humidity = process_measurement(humidity_buffer,tmp_humidity);

#if DEBUG_ENABLED
  Serial.print("Instant temperature: ");
  Serial.println(tmp_temperature);
  Serial.print("Averaged temperature: ");
  Serial.println(temperature);
  Serial.print("Instant humidity: ");
  Serial.println(tmp_humidity);
  Serial.print("Averaged humidity: ");
  Serial.println(humidity);
#endif
}

/* Update the polling period depending on the status of the system */
void update_polling_period()
{
  if(isnan(temperature) || isnan(humidity))
  {
    #if DEBUG_ENABLED
    Serial.println("Polling period in WARNING mode");
    #endif
    polling_period = WARNING_POLLING_PERIOD;
  }
  else
  {
    #if DEBUG_ENABLED
    Serial.println("Polling period in DEFAULT mode");
    #endif
    polling_period = DEFAULT_POLLING_PERIOD;
  }
}

/* Poll all the sensors and store their value in local variables */
void poll_sensors()
{
  /* Get temperature and humidity */
  get_temperature();

  /* Get the amount of light (only if request received) */
  if(lamp_request.state == "ON_LIGHT") get_light_amount();

  /* Update polling period */
  update_polling_period();
}

/* Node-specific logic */
/* Send light commands via IR */
/* Check if the light amount went above/below the threshold */
void node_logic()
{
  /* Handle light request */
  if(lamp_request.state == "ON_LIGHT" && (light_amount < LIGHT_THRESHOLD))
  {
#if DEBUG_ENABLED
    Serial.println("Sending ON command under low light conditions");
#endif
    RGB_to_IR ir_sender(IR_EMITTER_PIN);
    ir_sender.send_ir_on();
  }
  else if(lamp_request.state == "ON")
  {
#if DEBUG_ENABLED
    Serial.println("Sending ON command");
#endif
    RGB_to_IR ir_sender(IR_EMITTER_PIN);
    ir_sender.send_ir_on();
  }
  else if(lamp_request.state == "OFF")
  {
#if DEBUG_ENABLED
    Serial.println("Sending OFF command");
#endif
    RGB_to_IR ir_sender(IR_EMITTER_PIN);
    ir_sender.send_ir_off();
  }
  if(lamp_request.color)
  {
#if DEBUG_ENABLED
    Serial.println("Sending color command");
#endif
    RGB_to_IR ir_sender(IR_EMITTER_PIN);
    ir_sender.send_ir_rgb(lamp_request.R, lamp_request.G, lamp_request.B);
  }
}

/* Publish sensor information */
void publish_status()
{
#if DEBUG_ENABLED
  Serial.println("Publishing messages");
#endif
  /* Publish temperature and humidity with thanges largen than 10% */
  if( abs(temperature - temperature_old) > abs(MEASUREMENT_DELTA*temperature) )
  {
    #if DEBUG_ENABLED
    Serial.println("Detected change in temperature larger than 10%. Publishing temperature");
    #endif
    client.publish("attic_node/temperature", String(temperature).c_str() );
  }

  if( abs(humidity - humidity_old) > abs(MEASUREMENT_DELTA*humidity) )
  {
    #if DEBUG_ENABLED
    Serial.println("Detected change in humidity larger than 10%. Publishing humidity");
    #endif
    client.publish("attic_node/humidity", String(humidity).c_str() );
  }

}

void loop() {

  /* Connection handling */
  network_loop();

  long now = millis();
  if( (now - last_iteration) > polling_period )
  {
    last_iteration = now;

    /* Sensor polling */
    poll_sensors();

    /* Internal logic */
    node_logic();

    /* Publish sensor information */
    publish_status();
  }
}
