/*
  Home automation - entrance node

  Data gathering:
    Door status
    Presence

  Data publishing:
    Door status
    Entrance detected

  Topic subscription:
    entrance/warning

  Logic:
    Distinguish between entrance and exit
    Make sound if exit detected and warning message received


*/

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

/* Network settings */
const char* ssid = "";
const char* password = "";
const char* mqtt_server = "192.168.2.118";

/* Hardware settings */
#define DOOR_STATUS_PIN D4
#define PRESENCE_SENSOR_PIN D1
#define BUZZER_PIN D8
#define PRESENCE_COUNTER_MAX 10
#define PRESENCE_THRESHOLD 1000 //milliseconds
#define ALARM_DURATION 1 //repetitions
#define POLLING_PERIOD 3000 //milliseconds

#define DEBUG_ENABLED 1

/* Logical states */
bool door_status_open = false;
bool door_status_open_old = false;
bool warning_status = false;
bool entrance_detected = false;
unsigned long last_presence_detected = 0;

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



  //JSONencoder["door_open"] = 0;
  //JSONencoder["entrance_detected"] = 0;
}

/* Subscribe to the MQTT topics */
void setup_mqtt()
{
  /* Define MQTT broker */
  client.setServer(mqtt_server, 1883);
  /* Define callback function */
  client.setCallback(callback);
  /* Subscribe to topics */
  client.subscribe("entrance/warning");
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
 pinMode(DOOR_STATUS_PIN, INPUT);
 pinMode(BUZZER_PIN, OUTPUT);
 pinMode(PRESENCE_SENSOR_PIN, INPUT);

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
  if ((char)payload[0] == '1') warning_status = true;
  else warning_status = false;
}

/* Reconnect to the MQTT broker */
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("EntranceClient")) {
      Serial.println("connected");
      client.subscribe("entrance/warning");
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

/* Get the status of the door */
void get_door_status()
{
  /* Poll actual sensor */
  int polled_value = digitalRead(DOOR_STATUS_PIN);

  /* Save old status for change detection */
  door_status_open_old = door_status_open;

  if(polled_value) door_status_open = true;
  else door_status_open = false;

}

/* Get the status of the presence sensor and update presence counter */
void get_presence_status()
{

  int polled_value = digitalRead(PRESENCE_SENSOR_PIN);

  if(polled_value)
  {
    last_presence_detected = millis();
    #if DEBUG_ENABLED
    Serial.println("Presence detected");
    #endif
  }

}

/* Poll all the sensors and store their value in local variables */
void poll_sensors()
{
  /* Get the status of the door */
  get_door_status();

  /* Get the status of the presence detector */
  get_presence_status();
}

/* Make sound with the BUZZER_PIN */
void trigger_alarm()
{
  int duration = ALARM_DURATION;

  while(duration > 0)
  {
    /* Trigger BUZZER_PIN */
    digitalWrite(BUZZER_PIN,LOW);
    delay(2000);
    digitalWrite(BUZZER_PIN,HIGH);
    delay(1000);
    digitalWrite(BUZZER_PIN,LOW);
    delay(200);
    digitalWrite(BUZZER_PIN,HIGH);
    delay(100);
    digitalWrite(BUZZER_PIN,LOW);
    delay(200);
    digitalWrite(BUZZER_PIN,HIGH);
    delay(100);
    digitalWrite(BUZZER_PIN,LOW);
    delay(200);
    digitalWrite(BUZZER_PIN,HIGH);
    delay(10);
    digitalWrite(BUZZER_PIN,LOW);
    delay(20);
    digitalWrite(BUZZER_PIN,HIGH);
    delay(10);
    digitalWrite(BUZZER_PIN,LOW);
    duration--;
  }
}

/* Node-specific logic */
/* If presence has been detected before door opens --> somebody left */
/* If presence has been detected after door opens --> somebody came in */
void node_logic()
{
  /* Distinguish between entrance and exit */
  if(door_status_open && !door_status_open_old) //Only with status update
  {

    unsigned long time_since_presence = millis() - last_presence_detected();
    #if DEBUG_ENABLED
    Serial.print("Time since last presence detection (seconds): ");
    Serial.println(time_since_presence / 1000);
    #endif
    if(time_since_presence > PRESENCE_THRESHOLD)
    {
      entrance_detected = true;
      #if DEBUG_ENABLED
      Serial.println("Entrance detected");
      #endif
    }
  }
  else entrance_detected = false;

  /* Trigger alarm in case of warning status */
  if(warning_status && door_status_open) trigger_alarm();
}

/* Publish sensor information */
void publish_status()
{
  /* Publish door open information with status update */
  if(door_status_open != door_status_open_old)
  {
#if DEBUG_ENABLED
    Serial.print("Change in door status: ");
    Serial.println(door_status_open);
#endif
    client.publish("entrance_node/door_status", String(door_status_open).c_str());
  }

  /* Publish entrance information when detected */
  if(entrance_detected) client.publish("entrance_node/entrance_detection", String(entrance_detected).c_str());

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
