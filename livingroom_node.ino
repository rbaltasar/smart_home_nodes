/*
  Home automation - livingroom node

  Data gathering:
    Temperature & humidity
    Light amount

  Data publishing:
    Temperature & humidity

  Topic subscription:
    livingroom_node/light_state
    livingroom_node/light_color
    livingroom_node/light_intensity

  Node description:
    The node gathers and publishes temperature, humidity and light amount information
    The node receives light requests in three different ways:
      * MQTT topic
      * WebSocket request (Sinric --> Alexa)
    The node publishes the current state of the light (ON/OFF)
*/

/* Includes */
#include <ESP8266WiFiMulti.h>
#include <WebSocketsClient.h>
#include <WiFiClient.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <StreamString.h>
#include "RGB_to_IR.h"
#include <DHTesp.h>
#include <Wire.h>
#include <BH1750.h>

/* Network settings */
const char* ssid = "";
const char* password = "";

/* MQTT settings */
const char* mqtt_server = "192.168.2.118";

/* Sinric settings */
#define API_KEY "" // TODO: Change to your sinric API Key. Your API Key is displayed on sinric.com dashboard
#define DEVICE_ID ""
#define SERVER_URL "iot.sinric.com"
#define SERVER_PORT 80
#define HEARTBEAT_INTERVAL 300000 // 5 Minutes
#define MINIMUM_PUBLISH_RATE 600000 //Publish each topic at least every X

/* Debugging */
#define DEBUG_ENABLED 1

/* Hardware settings */
#define TEMPERATURE_SENSOR_PIN D1
#define DHTTYPE DHT11   // DHT 11
#define LIGHT_SENSOR_PIN_1 D6
#define LIGHT_SENSOR_PIN_2 D7
#define IR_EMITTER_PIN D2
#define MEASUREMENT_DELTA 0.01 //Minimum change in measurements (%) to publish the data

/* Ensure a minimum publish rate */
#define MINIMUM_PUBLISH_RATE 600000 //Publish each topic at least every 10 minutes
#define NUM_PUBLISH_TOPICS 3
#define DEFAULT_POLLING_PERIOD 20000 //10 seconds
#define WARNING_POLLING_PERIOD 20000 //Currently not used

/* Logical states */
/* Signal smoothing filters */
/*
 * The signal smoothing filter is implemented as a circular buffer.
 * For simplicity, the last position of the buffer contains always the index
 * of the oldest element in the buffer.
 */
 #define BUFFER_SIZE 20
float temperature_buffer[BUFFER_SIZE+1];
float humidity_buffer[BUFFER_SIZE+1];

/* Temperature measurement (average after filtering) */
//Improvement: create a struct
float temperature = 0;
float temperature_old = 99999; //Just a large initial value. Different from temperature

/* Humidity measurement (average after filtering) */
//Improvement: create a struct
float humidity = 0;
float humidity_old = 99999;

/* Light measurement */
float light_amount = 0;
float light_amount_old = 99999;

/* Period to enter the node_specific_loop */
uint32_t polling_period = DEFAULT_POLLING_PERIOD;
unsigned long last_iteration = 0;

/* Ensure a minimum publish rate */
unsigned long last_publish[NUM_PUBLISH_TOPICS];

/* Web server states --- Sinric */
uint64_t heartbeatTimestamp = 0;
bool isConnected = false;
void setPowerStateOnServer(String deviceId, String value);

/* Lamp control features */
bool publish_lamp_feedback = false;
struct lamp_status
{
  bool command;
  bool state;
  bool color;
  uint8_t R;
  uint8_t G;
  uint8_t B;
  int8_t brightness;
};

lamp_status lamp_request;
lamp_status lamp_request_old;

/* Communication settings */
WiFiClient espClient;
PubSubClient client(espClient);
WebSocketsClient webSocket;
DynamicJsonBuffer jsonBuffer(50);

/* Temperature and humdity sensor */
DHTesp temperature_sensor;

/* Light sensor */
BH1750 lightMeter;

/* IR utilities */
RGB_to_IR ir_sender(IR_EMITTER_PIN);

/* Setup */
void setup() {

#if DEBUG_ENABLED
  Serial.begin(115200);
#endif

  setup_wifi();
  setup_sinric();
  setup_hardware();
  setup_mqtt();

  lamp_request.state = false;
  lamp_request.R = 0;
  lamp_request.G = 0;
  lamp_request.B = 0;

  last_publish[0] = 0; //Temperature
  last_publish[1] = 0; //Humidity
  last_publish[2] = 0; //Light

  /* Ensure that the circular buffer starts at the index 0 */
  temperature_buffer[BUFFER_SIZE] = 0;
  humidity_buffer[BUFFER_SIZE] = 0;

  /* Share the serial printer object for debugging functionalities */
  HardwareSerial* hwPrint;
  hwPrint = &Serial;
  ir_sender.configure(hwPrint);

  /* Publish a boot event for error tracking and debugging */
  bool publish_succeeded = false;
  while(!publish_succeeded)
  {
    publish_succeeded = client.publish("livingroom_node/boot", "");
    if(!publish_succeeded) network_loop();
    yield();
  }

  blink_led(50);
}

/* Subscribe to the MQTT topics */
void setup_mqtt()
{
  /* Define MQTT broker */
  client.setServer(mqtt_server, 1883);
  /* Define callback function */
  client.setCallback(callback);
  /* Subscribe to topics */
  //Improve: put them as define or constant
  client.subscribe("livingroom_node/light_state");
  client.subscribe("livingroom_node/light_intensity");
  client.subscribe("livingroom_node/light_color");
}

/* Connect to the wireless network */
void setup_wifi() {

  delay(10);
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

#if DEBUG_ENABLED
   Serial.println("Wifi setup completed");
#endif
}

/* Configure SINRIC */
void setup_sinric()
{
  // server address, port and URL
  webSocket.begin("iot.sinric.com", 80, "/");

  // event handler
  webSocket.onEvent(webSocketEvent);
  webSocket.setAuthorization("apikey", API_KEY);

  // try again every 5000ms if connection has failed
  webSocket.setReconnectInterval(5000);

#if DEBUG_ENABLED
   Serial.println("Sinric setup completed");
#endif
}

/* Configure the hardware */
void setup_hardware()
{
  /* Built in LED */
  pinMode(BUILTIN_LED, OUTPUT);     // Initialize the BUILTIN_LED pin as an output
  digitalWrite(LED_BUILTIN, HIGH); //This means led OFF

 /* Initialize I2C bus */
  Wire.begin(LIGHT_SENSOR_PIN_1,LIGHT_SENSOR_PIN_2);
  lightMeter.begin();

 /* Setup pins */
 temperature_sensor.setup(TEMPERATURE_SENSOR_PIN, DHTesp::DHT11);

#if DEBUG_ENABLED
  Serial.println("Hardware setup completed");
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

  /* Parse JSON object */
  JsonObject& root = jsonBuffer.parseObject(payload);

  /* Filter for topics */
  if( strcmp(topic,"livingroom_node/light_state") == 0 )
  {
    int rcv = root["req"];
    if(rcv) lamp_request.state = true;
    else lamp_request.state = false;
    lamp_request.command = true;
    Serial.println(rcv);

  }
  else if(strcmp(topic,"livingroom_node/light_intensity") == 0)
  {
    int rcv = root["req"];
    lamp_request.brightness = rcv;
    Serial.println(rcv);
  }

  else if(strcmp(topic,"livingroom_node/light_color") == 0)
  {
    lamp_request.R = root["r"];
    lamp_request.G = root["g"];
    lamp_request.B = root["b"];
    lamp_request.color = true;

    // Output to serial monitor
#if DEBUG_ENABLED
    Serial.println(lamp_request.R);
    Serial.println(lamp_request.G);
    Serial.println(lamp_request.B);
#endif
  }

  last_iteration = 0;
}

/* Reconnect to the MQTT broker */
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("LivingroomClient")) {
      Serial.println("connected");
      /* Resubscribe */
      client.subscribe("livingroom_node/light_state");
      client.subscribe("livingroom_node/light_intensity");
      client.subscribe("livingroom_node/light_color");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void blink_led(uint16_t delay_ms)
{
  digitalWrite(LED_BUILTIN, LOW);
  delay(delay_ms);
  network_loop();
  digitalWrite(LED_BUILTIN, HIGH);
  delay(delay_ms);
  network_loop();
  digitalWrite(LED_BUILTIN, LOW);
  delay(delay_ms);
  network_loop();
  digitalWrite(LED_BUILTIN, HIGH);
  delay(delay_ms);
  network_loop();
  digitalWrite(LED_BUILTIN, LOW);
  delay(delay_ms);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(delay_ms);
  network_loop();
  digitalWrite(LED_BUILTIN, LOW);
  delay(delay_ms);
  network_loop();
  digitalWrite(LED_BUILTIN, HIGH);
}

/* Handle the network part of the loop */
void network_loop()
{
#if DEBUG_ENABLED
  unsigned long current_time = millis();
#endif

  /* MQTT loop */
  if (!client.connected()) reconnect();
  client.loop();

  /* WebSocket loop */
  webSocket.loop();

  if(isConnected)
  {
    uint64_t now = millis();
    // Send heartbeat in order to avoid disconnections during ISP resetting IPs over night. Thanks @MacSass
    if((now - heartbeatTimestamp) > HEARTBEAT_INTERVAL)
    {
        heartbeatTimestamp = now;
        webSocket.sendTXT("H");
    }
  }

#if 0
  current_time = millis() - current_time;
  Serial.print("Duration network_loop: ");
  Serial.println(current_time);
#endif
}

/* Average an array. Remove oldest element. Add newest element. Filter out NaN values */
float smoothing_filter(float* measurement_buffer, float measurement)
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

/* Get the amount of light */
void get_light_amount()
{
  light_amount = lightMeter.readLightLevel();

#if DEBUG_ENABLED
  Serial.print("Light amount: ");
  Serial.println(light_amount);
#endif
}

/* Get temperature and humidity */
void get_temperature()
{
  /* Instant measurement */
  float tmp_humidity = temperature_sensor.getHumidity();
  float tmp_temperature = temperature_sensor.getTemperature();

  /* Filter and average */
  temperature = smoothing_filter(temperature_buffer,tmp_temperature);
  humidity = smoothing_filter(humidity_buffer,tmp_humidity);

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
/* If the average measurement after filtering is NaN, reduce the polling period */
/* This feature is currently not used */
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

  /* Get the amount of light */
  get_light_amount();

  /* Update polling period */
  //update_polling_period();
}

/* Node-specific logic */
void node_logic()
{

  /* Process lamp state change request */
  if(lamp_request.command && (lamp_request.state != lamp_request_old.state))
  {
    if(lamp_request.state) ir_sender.send_ir_on();
    else ir_sender.send_ir_off();
    lamp_request_old.state = lamp_request.state;
    lamp_request.command = false;
    publish_lamp_feedback = true;

#if DEBUG_ENABLED
    Serial.println("Sending ON/OFF command");
#endif
  }

  /* Process color request */
  if(lamp_request.color)
  {
    ir_sender.send_ir_rgb(lamp_request.R, lamp_request.G, lamp_request.B);
    lamp_request.color = false;
    #if DEBUG_ENABLED
    Serial.println("Sending color command");
    #endif
  }

  /* Process brightness request */
  if(lamp_request.brightness != 0)
  {
    if(lamp_request.brightness > 0) ir_sender.send_ir_brightness(true);
    else ir_sender.send_ir_brightness(false);
    lamp_request.brightness = 0;
    #if DEBUG_ENABLED
    Serial.println("Sending brightness command");
    #endif
  }
}

/* Publish sensor information */
void publish_status()
{
#if DEBUG_ENABLED
  Serial.println("Publishing messages");
#endif

/* Compute time since last publish for each topic */
  unsigned long time_since_publish;
  bool publish_this[NUM_PUBLISH_TOPICS];
  for(uint8_t i=0; i<NUM_PUBLISH_TOPICS; i++)
  {
    time_since_publish = millis() - last_publish[i];
    if(time_since_publish > MINIMUM_PUBLISH_RATE) publish_this[i] = true;
    else publish_this[i] = false;
  }

  /* Publish temperature */
  if( (abs(temperature - temperature_old) > abs(MEASUREMENT_DELTA*temperature)) || publish_this[0])
  {
    #if DEBUG_ENABLED
    Serial.println("Detected change in temperature larger than 1%. Publishing temperature");
    #endif
    client.publish("livingroom_node/temperature", String(temperature).c_str() );
    temperature_old = temperature;
    last_publish[0] = millis();
  }

  /* Publish humidity */
  if( (abs(humidity - humidity_old) > abs(MEASUREMENT_DELTA*humidity)) || publish_this[1] )
  {
    #if DEBUG_ENABLED
    Serial.println("Detected change in humidity larger than 1%. Publishing humidity");
    #endif
    client.publish("livingroom_node/humidity", String(humidity).c_str() );
    humidity_old = humidity;
    last_publish[1] = millis();
  }

  /* Publish light amount */
  if( (abs(light_amount - light_amount_old) > abs(MEASUREMENT_DELTA*light_amount)) || publish_this[2] )
  {
    #if DEBUG_ENABLED
    Serial.println("Detected change in light amount larger than 1%. Publishing light amount");
    #endif
    client.publish("livingroom_node/light", String(light_amount).c_str() );
    light_amount_old = light_amount;
    last_publish[2] = millis();
  }

  /* Publish feedback of lamp state change */
  if(publish_lamp_feedback)
  {
    /* Ensure that the critical topics get published */
    bool publish_succeeded = false;
    while(!publish_succeeded)
    {
      publish_succeeded = client.publish("livingroom_node/lamp_feedback", String(lamp_request_old.state).c_str() );
      if(!publish_succeeded) network_loop();
      yield();
    }
    publish_lamp_feedback = false;
  }
}

/* Handle WebSocket event */
void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
  switch(type)
  {
    case WStype_DISCONNECTED:
    {
      isConnected = false;
      Serial.printf("[webSocketEvent] Webservice disconnected from server!\n");
      break;
    }
    case WStype_CONNECTED:
    {
      isConnected = true;
      Serial.printf("[webSocketEvent] Service connected to server at url: %s\n", payload);
      Serial.printf("[webSocketEvent] Waiting for commands from server ...\n");

      break;
    }
    case WStype_TEXT:
    {
      Serial.printf("[webSocketEvent] get text: %s\n", payload);
      DynamicJsonBuffer jsonBuffer;
      JsonObject& json = jsonBuffer.parseObject((char*)payload);
      String deviceId = json ["deviceId"];
      String action = json ["action"];

      if(deviceId != DEVICE_ID) return;

      if(action == "setPowerState") {
          String value = json ["value"];
          lamp_request.command = true;
          if(value == "ON") {
              lamp_request.state = true;
          } else {
              lamp_request.state = false;
          }
      }
      else if(action == "AdjustBrightness") {
          // alexa, dim lights  ==>{"deviceId":"xxx","action":"AdjustBrightness","value":-25}
          int16_t brightness = (int16_t)json["value"];
          lamp_request.brightness = brightness;
      }
      else if(action == "SetBrightness") {

      }
      else if(action == "SetColor") {
        double hue, saturation, brightness;
        hue = json ["value"]["hue"];
        saturation = json ["value"]["saturation"];
        brightness = json ["value"]["brightness"];

        ir_sender.HSV_to_RGB(hue,saturation,brightness,lamp_request.R,lamp_request.G,lamp_request.B);

        //if(hue > 180) lamp_request.B = 255;
        //else if(hue > 60) lamp_request.G = 255;
        //else lamp_request.R = 255;
        lamp_request.color = true;
      }
      else if(action == "IncreaseColorTemperature") {
         //alexa, set the lights softer ==> {"deviceId":"xxx","action":"IncreaseColorTemperature"}
      }
      else if(action == "IncreaseColorTemperature") {
         //alexa, set the lights softer ==> {"deviceId":"xxx","action":"IncreaseColorTemperature"}
      }
      else if(action == "SetColorTemperature") {
         //alexa, set the lights softer ==> {"deviceId":"xxx","action":"SetColorTemperature","value":2200}
        lamp_request.R = 255;
        lamp_request.G = 255;
        lamp_request.B = 255;
        lamp_request.color = true;
      }

      break;
    }
    case WStype_BIN:
    {
      Serial.printf("[webSocketEvent] get binary length: %u\n", length);
      break;
    }
  }
  last_iteration = 0;
}

/* Loop with node-specific stuff */
/* This shall include polling the sensors, doing any node-specifi logic and
 * publish the data.
 */
void node_specific_loop()
{
#if DEBUG_ENABLED
  /* Performance measurement */
  unsigned long current_time = millis();
#endif

  /* Sensor polling */
  poll_sensors();

  /* Internal logic */
  node_logic();

  /* Publish sensor information */
  publish_status();

#if 0
  current_time = millis() - current_time;
  Serial.print("Duration node loop: ");
  Serial.println(current_time);
#endif
}

void loop() {

  /* Connection handling */
  network_loop();

  long now = millis();
  if( (now - last_iteration) > polling_period )
  {
    /* Node specific loop */
    node_specific_loop();

    /* Update last iteration time */
    last_iteration = now;
  }
}
