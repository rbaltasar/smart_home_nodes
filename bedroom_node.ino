/*
  Home automation - bedtroom node

  Data gathering:
    Temperature & humidity

  Data publishing:
    Temperature & humidity

  Topic subscription:
    bedroom/light

  Logic:

*/

#include <ESP8266WiFiMulti.h>
#include <WebSocketsClient.h>
#include <WiFiClient.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <StreamString.h>
#include "RGB_to_IR.h"
#include <DHTesp.h>

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

/* Debugging */
#define DEBUG_ENABLED 0

/* Hardware settings */
#define TEMPERATURE_SENSOR_PIN D1
#define DHTTYPE DHT11   // DHT 11
#define IR_EMITTER_PIN D2
#define MEASUREMENT_DELTA 0.01

#define DEFAULT_POLLING_PERIOD 3000 //10 seconds
#define WARNING_POLLING_PERIOD 3000 //100 seconds

/* Data collection settings */
#define BUFFER_SIZE 50

/* Logical states */
float temperature_buffer[BUFFER_SIZE+1];
float humidity_buffer[BUFFER_SIZE+1];
float temperature = 0;
float temperature_old = 99999;
float humidity = 0;
float humidity_old = 99999;
uint8_t nan_count = 0;
uint32_t polling_period = DEFAULT_POLLING_PERIOD;

/* Web server states --- Sinric */
uint64_t heartbeatTimestamp = 0;
bool isConnected = false;
String page = "";
void setPowerStateOnServer(String deviceId, String value);

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
long last_iteration = 0;
char msg[5];
ESP8266WiFiMulti WiFiMulti;
WebSocketsClient webSocket;

DHTesp temperature_sensor;
RGB_to_IR ir_sender(IR_EMITTER_PIN);

void setup() {

  pinMode(BUILTIN_LED, OUTPUT);     // Initialize the BUILTIN_LED pin as an output
  Serial.begin(115200);

  setup_wifi();
  setup_sinric();
  setup_hardware();
  setup_mqtt();

  lamp_request.state = false;
  lamp_request.R = 0;
  lamp_request.G = 0;
  lamp_request.B = 0;

  temperature_buffer[BUFFER_SIZE] = 0;
  humidity_buffer[BUFFER_SIZE] = 0;

  HardwareSerial* hwPrint;
  hwPrint = &Serial;
  ir_sender.configure(hwPrint);
}

/* Subscribe to the MQTT topics */
void setup_mqtt()
{
  /* Define MQTT broker */
  client.setServer(mqtt_server, 1883);
  /* Define callback function */
  client.setCallback(callback);
  /* Subscribe to topics */
  client.subscribe("bedroom_node/light_state");
  client.subscribe("bedroom_node/light_intensity");
  client.subscribe("bedroom_node/light_color");
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

/* Configure SINRIC */
void setup_sinric()
{
  //server address, port and URL
  //webSocket.begin(SERVER_URL, SERVER_PORT, "/");
  webSocket.begin("iot.sinric.com", 80, "/");

  //event handler
  webSocket.onEvent(webSocketEvent);
  webSocket.setAuthorization("apikey", API_KEY);

  //try again every 5000ms if connection has failed
  webSocket.setReconnectInterval(5000);   // If you see 'class WebSocketsClient' has no member named 'setReconnectInterval' error update arduinoWebSockets

}

/* Configure the hardware */
void setup_hardware()
{

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

 /* Setup pins */
 temperature_sensor.setup(TEMPERATURE_SENSOR_PIN, DHTesp::DHT11);

#if DEBUG_ENABLED
   Serial.println("Hardware setup");
#endif
}

/* Configure the callback function for a subscribed topic */
void callback(char* topic, byte* payload, unsigned int length) {

  /* Print message (debugging only) */
#if 1
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
#endif

  /* Filter for topics (optional when >1 topics) */
  if( strcmp(topic,"bedroom_node/light_state") == 0 )
  {
    DynamicJsonBuffer jsonBuffer(500);
    JsonObject& root = jsonBuffer.parseObject(payload);
    // Parameters
    int rcv = root["req"];
    if(rcv) lamp_request.state = true;
    else lamp_request.state = false;
    lamp_request.command = true;
    Serial.println(rcv);

  }
  else if(strcmp(topic,"bedroom_node/light_intensity") == 0)
  {
    DynamicJsonBuffer jsonBuffer(500);
    JsonObject& root = jsonBuffer.parseObject(payload);
    // Parameters
    int rcv = root["req"];
    lamp_request.brightness = rcv;
    Serial.println(rcv);
  }

  else if(strcmp(topic,"bedroom_node/light_color") == 0)
  {
    DynamicJsonBuffer jsonBuffer(500);
    JsonObject& root = jsonBuffer.parseObject(payload);
      // Parameters
    int R = root["r"];
    int G = root["g"];
    int B = root["b"];

    lamp_request.R = R;
    lamp_request.G = G;
    lamp_request.B = B;
    lamp_request.color = true;

    // Output to serial monitor
    Serial.println(R);
    Serial.println(G);
    Serial.println(B);
  }
}

/* Reconnect to the MQTT broker */
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("BedroomClient")) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      //client.publish("outTopic", "hello world");
      // ... and resubscribe
      client.subscribe("bedroom_node/light_state");
      client.subscribe("bedroom_node/light_intensity");
      client.subscribe("bedroom_node/light_color");
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

  webSocket.loop();

  if(isConnected) {
      uint64_t now = millis();

      // Send heartbeat in order to avoid disconnections during ISP resetting IPs over night. Thanks @MacSass
      if((now - heartbeatTimestamp) > HEARTBEAT_INTERVAL) {
          heartbeatTimestamp = now;
          webSocket.sendTXT("H");
      }
  }
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

  /* Update polling period */
  update_polling_period();
}

/* Node-specific logic */
/* Send light commands via IR */
/* Check if the light amount went above/below the threshold */
void node_logic()
{

  if(lamp_request.command)
  {
    Serial.println("Difference in request state!");
    if(lamp_request.state) ir_sender.send_ir_on();
    else ir_sender.send_ir_off();
    #if DEBUG_ENABLED
    Serial.println("Sending ON/OFF command");
    #endif
    lamp_request_old.state = lamp_request.state;
    lamp_request.command = false;
  }

  if(lamp_request.color)
  {
    ir_sender.send_ir_rgb(lamp_request.R, lamp_request.G, lamp_request.B);
    lamp_request.color = false;
    #if DEBUG_ENABLED
    Serial.println("Sending color command");
    #endif
  }

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

  /* Publish temperature and humidity */
  if( abs(temperature - temperature_old) > abs(MEASUREMENT_DELTA*temperature) )
  {
    #if DEBUG_ENABLED
    Serial.println("Detected change in temperature larger than 1%. Publishing temperature");
    #endif
    client.publish("bedroom_node/temperature", String(temperature).c_str() );
    temperature_old = temperature;
  }

  if( abs(humidity - humidity_old) > abs(MEASUREMENT_DELTA*humidity) )
  {
    #if DEBUG_ENABLED
    Serial.println("Detected change in humidity larger than 1%. Publishing humidity");
    #endif
    client.publish("bedroom_node/humidity", String(humidity).c_str() );
    humidity_old = humidity;
  }
}

void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
  switch(type) {
    case WStype_DISCONNECTED:
      isConnected = false;
      Serial.printf("[webSocketEvent] Webservice disconnected from server!\n");
      break;
    case WStype_CONNECTED: {
      isConnected = true;
      Serial.printf("[webSocketEvent] Service connected to server at url: %s\n", payload);
      Serial.printf("[webSocketEvent] Waiting for commands from server ...\n");
      }
      break;
    case WStype_TEXT: {
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
          lamp_request.color = true;
        }
        else if(action == "IncreaseColorTemperature") {

        }
        else if(action == "IncreaseColorTemperature") {

        }
        else if(action == "SetColorTemperature") {
          lamp_request.R = 255;
          lamp_request.G = 255;
          lamp_request.B = 255;
          lamp_request.color = true;
        }
      }
      break;
    case WStype_BIN:
      Serial.printf("[webSocketEvent] get binary length: %u\n", length);
      break;
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
