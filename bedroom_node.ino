/*
  Home automation - bedroom node

  Data gathering:
    Temperature & humidity

  Data publishing:
    Temperature & humidity

  Topic subscription:
    bedroom/light
    bedroom/warning
    bedroom/wakeup

  Logic:

*/

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

/* Network settings */
const char* ssid = "........";
const char* password = "........";
const char* mqtt_server = "192.168.1.51";

/* Hardware settings */
#define TEMPERATURE_SENSOR_PIN 1
#define IR_EMITTER_PIN 3
#define POLLING_PERIOD 1000 //milliseconds

/* Logical states */
bool warning = false;
int warning_counter = 0;
bool wakeup = false;
bool lamp_on = false;
int temperature = 0;
int humidity = 0;

struct
{
  bool on;
  bool off;
  bool color;
  uint8_t R;
  uint8_t G;
  uint8_t B;
} lamp_info;

lamp_info lamp_request;

/* Communication settings */
WiFiClient espClient;
PubSubClient client(espClient);
long last_iteration = 0;
char msg[5];
//StaticJsonBuffer<200> jsonBuffer;
//JsonObject& JSONencoder = JSONbuffer.createObject();

void setup() {
  pinMode(BUILTIN_LED, OUTPUT);     // Initialize the BUILTIN_LED pin as an output
  Serial.begin(115200);

  setup_wifi();
  setup_hardware();
  setup_mqtt();

  lamp_request.on = false;
  lamp_request.off = false;
  lamp_request.R = 0;
  lamp_request.G = 0;
  lamp_request.B = 0;

}

/* Subscribe to the MQTT topics */
void setup_mqtt()
{
  /* Define MQTT broker */
  client.setServer(mqtt_server, 1883);
  /* Define callback function */
  client.setCallback(callback);
  /* Subscribe to topics */
  client.subscribe("bedroom/light");
  client.subscribe("bedroom/warning");
  client.subscribe("bedroom/wakeup");
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

}

/* Configure the callback function for a subscribed topic */
void callback(char* topic, byte* payload, unsigned int length) {

  /* Print message (debugging only) */
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  /* Filter for topics (optional when >1 topics) */

  /* Read topic info */

  /* Parse JSON message */

  /* Save request in local memory */
  lamp_request.on =
  lamp_request.off =
  lamp_request.color =
  if(lamp_request.color)
  {
    lamp_request.R =
    lamp_request.G =
    lamp_request.B =
  }

  if(warning_message == warning_off)
  {
    lamp_request.color = true;
    lamp_request.R = 255;
    lamp_request.G = 255;
    lamp_request.B = 255;
  }
  warning =

  wakeup =
}

/* Reconnect to the MQTT broker */
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP8266Client")) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("outTopic", "hello world");
      // ... and resubscribe
      client.subscribe("inTopic");
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

/* Get temperature and humidity */
void get_temperature()
{
  //TODO
}

/* Poll all the sensors and store their value in local variables */
void poll_sensors()
{
  /* Get temperature and humidity */
  get_temperature();
}

/* Node-specific logic */
/* Send light commands via IR */
/* Check if the light amount went above/below the threshold */
void node_logic()
{
  /* Handle light request */
  if(lamp_request.on)
  {
    RGB_to_IR ir_sender(IR_EMITTER_PIN);
    ir_sender.send_ir_on();
    lamp_on = true;
  }
  else if(lamp_request.off)
  {
    RGB_to_IR ir_sender(IR_EMITTER_PIN);
    ir_sender.send_ir_off();
    lamp_on = false;
  }
  if(lamp_request.color && lamp_on)
  {
    RGB_to_IR ir_sender(IR_EMITTER_PIN);
    ir_sender.send_ir_rgb(lamp_request.R, lamp_request.G, lamp_request.B);
  }

  /* Handle warning request */
  if(lamp_on && warning)
  {
    if(warning_counter == 0)
    {
      warning_counter++;
      RGB_to_IR ir_sender(IR_EMITTER_PIN);
      ir_sender.send_ir_rgb(255, 0, 0);
    }
    else
    {
      warning_counter--;
      RGB_to_IR ir_sender(IR_EMITTER_PIN);
      ir_sender.send_ir_rgb(0, 0, 0);
    }
  }
  /* Handle wakeup request */
  else if(wakeup)
  {
    RGB_to_IR ir_sender(IR_EMITTER_PIN);
    //Switch ON
    ir_sender.send_ir_on();
    //Set morning color
    ir_sender.send_ir_rgb(255,255,0);
  }

}

/* Publish sensor information */
void publish_status()
{
  /* Publish temperature and humidity */

}

void loop() {

  /* Connection handling */
  network_loop();

  long now = millis();
  if( (now - last_iteration) > POLLING_PERIOD )
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
